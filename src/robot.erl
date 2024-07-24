-module(robot).

-export([robot_init/1,modify_frequency/1]).

-record(cal, {acc, gyro}).

-define(RAD_TO_DEG, 180.0/math:pi()).
-define(DEG_TO_RAD, math:pi()/180.0).

%Advance constant
-define(ADV_V_MAX, 30.0).

%Turning constant
-define(TURN_V_MAX, 80.0).

%Angle at which robot is considered "down"
-define(MAX_ANGLE, 25.0).

%Coefficient for the complementary filter
-define(COEF_FILTER, 0.667).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Robot 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

robot_init(Hera_pid) ->

    process_flag(priority, max),

    %Starting timestamp
    T0 = erlang:system_time()/1.0e6,

    %Table for global variables
    ets:new(variables, [set, public, named_table]),
    ets:insert(variables, {"Freq_Goal", 300.0}),

    %Calibration
    io:format("Calibrating... Do not move the pmod_nav!~n"),
    grisp_led:color(1, {1, 0, 0}),
    grisp_led:color(2, {1, 0, 0}),
    [_Gx0,Gy0,_Gz0] = calibr_mes:calibrate(),
    io:format("Done calibrating~n"),

    %Kalman matrices
    X0 = mat:matrix([[0], [0]]),
    P0 = mat:matrix([[0.1, 0], [0, 0.1]]),

    %I2C bus
    I2Cbus = grisp_i2c:open(i2c1),

    %PIDs initialisation
    Pid_Speed = spawn(pid_controller, pid_init, [-0.12, -0.07, 0.0, -1, 60.0, 0.0]),
    Pid_Stability = spawn(pid_controller, pid_init, [17.0, 0.0, 4.0, -1, -1, 0.0]),
    io:format("Pid of the speed controller: ~p.~n", [Pid_Speed]),
    io:format("Pid of the stability controller: ~p.~n", [Pid_Stability]),
	io:format("Starting movement of the robot.~n"),

    %Call main loop
    robot_main(T0, Hera_pid, {0.0, rest, false}, {T0, X0, P0}, I2Cbus, {0, T0, []}, {Gy0, 0.0, 0.0}, {Pid_Speed, Pid_Stability}, {0.0, 0.0}, {0, 0, 200.0, T0}).

robot_main(Start_Time, Hera_pid, {End_Timer_Freefall, Robot_State, Robot_Up_Check}, {T0, X0, P0}, I2Cbus, {Logging, Log_End, Log_List}, {Gy0, Angle_Complem, Angle_Rate}, {Pid_Speed, Pid_Stability}, {Adv_V_Ref, Turn_V_Ref}, {N, Freq, Mean_Freq, T_End}) ->

    %Delta time of loop
    T1 = erlang:system_time()/1.0e6, %[ms]
	Dt = (T1- T0)/1000.0,            %[s]

    %%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Input from Sensor %%%
    %%%%%%%%%%%%%%%%%%%%%%%%%

    %Read data
    [Gy,Ax,_Ay,Az] = pmod_nav:read(acc, [out_y_g, out_x_xl, out_y_xl, out_z_xl], #{g_unit => dps}),

    %%%%%%%%%%%%%%%%%%%%%%%%
    %%% Input from ESP32 %%%
    %%%%%%%%%%%%%%%%%%%%%%%%

    %Receive I2C and conversion
    [<<SL1,SL2,SR1,SR2,CtrlByte>>] = grisp_i2c:transfer(I2Cbus, [{read, 16#40, 1, 5}]),
	[Speed_L,Speed_R] = hera_com:decode_half_float(<<SL1,SL2,SR1,SR2>>),
    Speed = (Speed_L + Speed_R)/2,

    %Retrieve flags from ESP32
    [Arm_Ready, Switch, Test, Get_Up, Forward, Backward, Left, Right] = hera_com:get_bits(CtrlByte),

    %%%%%%%%%%%%%%%%%%%%%
    %%% Command Logic %%%
    %%%%%%%%%%%%%%%%%%%%%

    %Set advance speed from flags
    Adv_V_Goal = speed_ref(Forward, Backward),

    %Set turning speed from flags
    Turn_V_Goal = turn_ref(Left, Right),

    %%%%%%%%%%%%%%%%%%%%%%%%%%
    %%% Angle Computations %%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%

    %Angle based directly on the sensors
    Angle_Accelerometer = math:atan(Az / (-Ax))*?RAD_TO_DEG,

    %Kalman filter computation
    {X1, P1} = kalman_angle(Dt, Ax, Az, Gy, Gy0, X0, P0),
	[Th_Kalman, _W_Kalman] = mat:to_array(X1),
    Angle_Kalman = Th_Kalman*?RAD_TO_DEG,

    %Complementary angle computation
    K = 1.25/(1.25+(1.0/Mean_Freq)),
    {Angle_Complem_New, Angle_Rate_New} = complem_angle({Dt, Ax, Az, Gy, Gy0, K, Angle_Complem, Angle_Rate}),

    %Select angle between kalman or complementary
    Angle = select_angle(Switch, Angle_Kalman, Angle_Complem),


    %%%%%%%%%%%%%%%%%%
    %%% Controller %%%
    %%%%%%%%%%%%%%%%%%

    %Takes as input:
    % -Measures
    % -Process IDs of the two PID controllers
    % -Velocity to reach and loopback velocity of trapezoidal profile
    {Acc, Adv_V_Ref_New, Turn_V_Ref_New} = stability_engine:controller({Dt, Angle, Speed}, {Pid_Speed, Pid_Stability}, {Adv_V_Goal, Adv_V_Ref}, {Turn_V_Goal, Turn_V_Ref}),
    %Gives as output:
    % -Acceleration of the motors -> sent to ESP32 
    % -Loopback V_Ref for advance
    % -Loopback V_Ref for turning -> sent to ESP32 

    %%%%%%%%%%%%%%%%%%%%%%%
    %%% Output to ESP32 %%%
    %%%%%%%%%%%%%%%%%%%%%%%

    %Timer for free fall
    if 
        (Robot_State == prepare_arms) and Arm_Ready -> End_Timer_Freefall_New = erlang:system_time()/1.0e6 + 1000;
        true -> End_Timer_Freefall_New = End_Timer_Freefall
    end,

    %State of the robot
    if 
        Robot_Up_Check and (abs(Angle) > 20.0) ->
            Robot_Up_Check_New = false;
        not Robot_Up_Check and (abs(Angle) < 18.0) -> 
            Robot_Up_Check_New = true;
        true ->
            Robot_Up_Check_New = Robot_Up_Check
    end,
    if
        Angle > 0.0 ->
            F_B = 1;
        true ->
            F_B = 0
    end,

    %State change
    case Robot_State of
        rest -> 
            if
                Get_Up -> Next_Robot_State = raising;
                true -> Next_Robot_State = rest
            end;
        raising -> 
            if
                Robot_Up_Check -> Next_Robot_State = stand_up;
                not Get_Up -> Next_Robot_State = soft_fall;
                true -> Next_Robot_State = raising
            end;
        stand_up -> 
            if
                not Get_Up -> Next_Robot_State = wait_for_extend;
                not Robot_Up_Check -> Next_Robot_State = rest;
                true -> Next_Robot_State = stand_up
            end;
        wait_for_extend -> 
            Next_Robot_State = prepare_arms;
        prepare_arms -> 
            if
                Arm_Ready -> Next_Robot_State = free_fall;
                Get_Up -> Next_Robot_State = stand_up;
                not Robot_Up_Check -> Next_Robot_State = rest;
                true -> Next_Robot_State = prepare_arms
            end;
        free_fall -> 
            Time = erlang:system_time()/1.0e6,
            if
                Time > End_Timer_Freefall_New -> Next_Robot_State = wait_for_retract;
                true -> Next_Robot_State = free_fall
            end;
        wait_for_retract ->
            Next_Robot_State = soft_fall;
        soft_fall -> 
            if
                Arm_Ready -> Next_Robot_State = rest;
                Get_Up -> Next_Robot_State = raising;
                true -> Next_Robot_State = soft_fall
            end
    end,

    %State output
    case Next_Robot_State of
        rest -> 
            Power    = 0,
            Freeze   = 0,
            Extend   = 0,
            Robot_Up = 0;
        raising -> 
            Power    = 1,
            Freeze   = 0,
            Extend   = 1,
            Robot_Up = 0;
        stand_up -> 
            Power    = 1,
            Freeze   = 0,
            Extend   = 0,
            Robot_Up = 1;
        wait_for_extend -> 
            Power    = 1,
            Freeze   = 0,
            Extend   = 1,
            Robot_Up = 1;
        prepare_arms -> 
            Power    = 1,
            Freeze   = 0,
            Extend   = 1,
            Robot_Up = 1;
        free_fall -> 
            Power    = 1,
            Freeze   = 1,
            Extend   = 1,
            Robot_Up = 1;
        wait_for_retract -> 
            Power    = 1,
            Freeze   = 0,
            Extend   = 0,
            Robot_Up = 0;
        soft_fall -> 
            Power    = 1,
            Freeze   = 0,
            Extend   = 0,
            Robot_Up = 0
    end,

    %Send output to ESP32
    Output_Byte = get_byte([Power, Freeze, Extend, Robot_Up, F_B, 0, 0, 0]),
    [HF1, HF2] = hera_com:encode_half_float([Acc, Turn_V_Ref_New]),
    grisp_i2c:transfer(I2Cbus, [{write, 16#40, 1, [HF1, HF2, <<Output_Byte>>]}]),

    %%%%%%%%%%%%%%%%%%%%%%%%
    %%% Testing Features %%%
    %%%%%%%%%%%%%%%%%%%%%%%%

    %Frequency computation
    {N_New, Freq_New, Mean_Freq_New} = frequency_computation(Dt, N, Freq, Mean_Freq),

    %Logging 
    if 
        Test ->
            Log_End_New = erlang:system_time()/1.0e6 + 6000.0;
        true -> 
            Log_End_New = Log_End
    end,
    Logging_New = erlang:system_time()/1.0e6 < Log_End_New,

    %LED flickering while logging
    if 
        Logging_New -> 
            if 
                N rem 9 < 4 -> 
                    grisp_led:color(1, {1, 1, 0}),
                    grisp_led:color(2, {1, 1, 0});
                true->
                    grisp_led:color(1, {0, 0, 0}),
                    grisp_led:color(2, {0, 0, 0})
            end;
        true ->
            ok
    end,

    %Check for start or end of logging sequence
    if
        not Logging and Logging_New ->
            Hera_pid ! {self(), start_log};
        Logging and not Logging_New ->
            grisp_led:color(1, {1, 1, 0}),
            grisp_led:color(2, {1, 1, 0}),
            Hera_pid ! {self(), stop_log};
        true ->
            ok
    end,

    %Send values to ESP32 if asked, else stack up values in list
    receive
        {From, log_values} ->    
            From ! {self(), Log_List},
            Log_List_New = []
    after 0 ->
        if
            Logging_New == 1.0 ->
                Log_List_New = lists:append(Log_List, [T1-Start_Time, Mean_Freq, Gy, Acc, CtrlByte, Angle_Accelerometer, Angle_Kalman, Angle_Complem, Adv_V_Ref, Switch]);
            true ->
                Log_List_New = Log_List
        end
    end,

    %Communication with Hera (more messages can be implemented by the user)
    receive
        {From1, freq} -> From1 ! {self(), 1/Dt};
        {From2, acc} -> From2 ! {self(), Acc};
        {_, Msg} -> io:format("Message [~p] not recognized. ~nPossible querries are: [freq, acc]. ~nMore querries can be added.", [Msg])
    after 0 ->
        ok
    end,

    %Imposed maximum frequency
    T2 = erlang:system_time()/1.0e6,
    [{_,Freq_Goal}] = ets:lookup(variables, "Freq_Goal"),
    Delay_Goal = 1.0/Freq_Goal * 1000.0,
    if
        T2-T_End < Delay_Goal ->
            wait(Delay_Goal-(T2-T1));
        true ->
            ok
    end,
    T_End_New = erlang:system_time()/1.0e6,

    %Loop back with updated state
    robot_main(Start_Time, Hera_pid, {End_Timer_Freefall_New, Next_Robot_State, Robot_Up_Check_New}, {T1, X1, P1}, I2Cbus, {Logging_New, Log_End_New, Log_List_New}, {Gy0, Angle_Complem_New, Angle_Rate_New}, {Pid_Speed, Pid_Stability}, {Adv_V_Ref_New, Turn_V_Ref_New}, {N_New, Freq_New, Mean_Freq_New, T_End_New}).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Internal functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

calibrate() ->
    Data = [list_to_tuple(pmod_nav:read(Comp, Registers)) || _ <- lists:seq(1,N)],
    {X, Y, Z} = lists:unzip3(Data),
    [lists:sum(X)/N, lists:sum(Y)/N, lists:sum(Z)/N]. %[Gx0, Gy0, Gz0]

kalman_angle(Dt, Ax, Az, Gy, Gy0, X0, P0) ->
    R = mat:matrix([[3.0, 0.0], [0, 3.0e-6]]),
    Q = mat:matrix([[3.0e-5, 0.0], [0.0, 10.0]]),
    F = fun (X) -> [Th, W] = mat:to_array(X),
				mat:matrix([ 	[Th+Dt*W],
								[W      ] ])
		end,
    Jf = fun (X) -> [_Th, _W] = mat:to_array(X),
				mat:matrix([  	[1, Dt],
								[0, 1 ] ])
		 end,
    H = fun (X) -> [Th, W] = mat:to_array(X),
				mat:matrix([ 	[Th],
								[W ] ])
		end,
    Jh = fun (X) -> [_Th, _W] = mat:to_array(X),
				mat:matrix([  	[1, 0],
								[0, 1] ])
		 end,
    Z = mat:matrix([[math:atan(Az / (-Ax))], [(Gy-Gy0)*?DEG_TO_RAD]]),
    kalman:ekf({X0, P0}, {F, Jf}, {H, Jh}, Q, R, Z).

complem_angle({Dt, Ax, Az, Gy, Gy0, K, Angle_Complem, Angle_Rate}) ->

    Angle_Rate_New = (Gy - Gy0) * ?COEF_FILTER + Angle_Rate * (1 - ?COEF_FILTER),

    %Angle increment computed from gyroscope
    Delta_Gyr = Angle_Rate_New * Dt,

    %Absolute angle computed form accelerometer
    Angle_Acc = math:atan(Az / (-Ax)) * 180 / math:pi(),

    %Complementary filter combining gyroscope and accelerometer
    Angle_Complem_New = (Angle_Complem + Delta_Gyr) * K + Angle_Acc * (1 - K), 
    
    {Angle_Complem_New, Angle_Rate_New}.

select_angle(Switch, Angle_Kalman, Angle_Complem) ->
    if
        Switch -> 
            Angle = Angle_Kalman;
        true ->
            Angle = Angle_Complem
    end,
    Angle.

speed_ref(Forward, Backward) ->
    if
        Forward ->
            Adv_V_Goal = ?ADV_V_MAX;
        Backward ->
            Adv_V_Goal = - ?ADV_V_MAX;
        true ->
            Adv_V_Goal = 0.0
    end,
    Adv_V_Goal.

turn_ref(Left, Right) ->
    if
        Right ->
            Turn_V_Goal = ?TURN_V_MAX;
        Left ->
            Turn_V_Goal = - ?TURN_V_MAX;
        true ->
            Turn_V_Goal = 0.0
    end,
    Turn_V_Goal.


frequency_computation(Dt, N, Freq, Mean_Freq) ->
    if 
        N == 100 ->
            erlang:display(Freq),
            N_New = 0,
            Freq_New = 0,
            Mean_Freq_New = Freq,

            %Reset LED color after some time
            grisp_led:color(1, {1, 1, 0}),
	        grisp_led:color(2, {1, 1, 0});
        true ->
            N_New = N+1,
            Freq_New = ((Freq*N)+(1/Dt))/(N+1),
            Mean_Freq_New = Mean_Freq
    end,
    {N_New, Freq_New, Mean_Freq_New}.

cut_motors(Angle) ->
    if  
        abs(Angle) > ?MAX_ANGLE ->
            Pause = 0.0;
        true ->
            Pause = 1.0  
    end,
    Pause.

wait(T) -> 
	Tnow = erlang:system_time()/1.0e6,
	wait_help(Tnow,Tnow+T).
wait_help(Tnow, Tend) when Tnow >= Tend -> ok;
wait_help(_, Tend) -> 
    Tnow = erlang:system_time()/1.0e6,
    wait_help(Tnow,Tend).

get_byte(List) ->
    [A, B, C, D, E, F, G, H] = List,
    A*128 + B*64 + C*32 + D*16 + E*8 + F*4 + G*2 + H. 


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Global variables
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

modify_frequency(Freq) ->
    ets:insert(variables, {"Freq_Goal", Freq}),
    ok.
