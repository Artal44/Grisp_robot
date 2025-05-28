-module(main_loop).

-export([robot_init/0]).

-define(RAD_TO_DEG, 180.0/math:pi()).
-define(DEG_TO_RAD, math:pi()/180.0).

-define(ADV_V_MAX, 30.0).
-define(TURN_V_MAX, 80.0).
-define(g, 9.81).
-define(M, 3.4).
-define(h, 0.26).
-define(I, 0.26).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% INITIALISATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

robot_init() ->

    process_flag(priority, max),

    calibrate(),

    {X0, P0} = init_kalman(),
    {Old_X0, Old_P0} = old_init_kalman(),

    %I2C bus
    I2Cbus = grisp_i2c:open(i2c1),
    persistent_term:put(i2c, I2Cbus),

    %PIDs initialisation
    Pid_Speed = spawn(pid_controller, pid_init, [-0.12, -0.07, 0.0, -1, 60.0, 0.0]),
    Pid_Stability = spawn(pid_controller, pid_init, [20, 0.0, 5.8, -1, -1, 0.0]), % 20.4 pour Kp et 5.8
    persistent_term:put(controllers, {Pid_Speed, Pid_Stability}),
    persistent_term:put(freq_goal, 300.0),

    T0 = erlang:system_time()/1.0e6,
    
	io:format("[ROBOT] Robot ready.~n"),

    State = #{
        robot_state => {rest, false}, %{Robot_State, Robot_Up}
        kalman_state => {T0, X0, P0}, %{Tk, Xk, Pk}
        old_kalman_state => {T0, Old_X0, Old_P0}, %{Tk, Xk, Pk}
        move_speed => {0.0, 0.0}, % {Adv_V_Ref, Turn_V_Ref}
        frequency => {0, 0, 200.0, T0}, %{N, Freq, Mean_Freq, T_End}
	    acc_prev => 0.0
    }, 

    robot_loop(State).

robot_loop(State) ->
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PARSE STATE MAP %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
    {Robot_State, Robot_Up} = maps:get(robot_state, State),
    {Tk, Xk, Pk} = maps:get(kalman_state, State),
    {Old_Tk, Old_Xk, Old_Pk} = maps:get(old_kalman_state, State),
    {Adv_V_Ref, Turn_V_Ref} = maps:get(move_speed, State),
    {N, Freq, Mean_Freq, T_End} = maps:get(frequency, State), 
    Acc_Prev = maps:get(acc_prev, State),

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% COMPUTE Dt BETWEEN ITERATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    T1 = erlang:system_time()/1.0e6,
    Dt = (T1- Tk)/1000.0,
    Old_Dt = (T1 - Old_Tk)/1000.0,

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% GET NEW PMOD_NAV MEASURE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    [Gy,Ax,Az] = pmod_nav:read(acc, [out_y_g, out_x_xl, out_z_xl], #{g_unit => dps}),

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% GET INPUT FROM I2CBus %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    {Speed, CtrlByte} = i2c_read(),
    [Arm_Ready, _, _, Get_Up, Forward, Backward, Left, Right] = hera_com:get_bits(CtrlByte),

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% DERIVE CONTROLS FROM INPUTS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Adv_V_Goal = speed_ref(Forward, Backward),
    Turn_V_Goal = turn_ref(Left, Right),

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% OLD KALMAN COMPUTATIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    [Old_Angle, {Old_X1, Old_P1}] = old_kalman_angle(Old_Dt, Ax, Az, Gy, Old_Xk, Old_Pk),

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% KALMAN COMPUTATIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Acc_Prev_Rad = Acc_Prev * math:pi() / 180.0, % Only if needed!
    [Angle, {X1, P1}] = kalman_angle(Dt, Ax, Az, Gy, Acc_Prev_Rad, Xk, Pk),

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% MEASURED DIRECT ANGLE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Direct_angle = math:atan(Az / (-Ax)) * ?RAD_TO_DEG,
    io:format("[ROBOT] NEW Kalman=~.2f, OLD Kalman=~.2f, Measured=~.2f~n", [Angle, Old_Angle, Direct_angle]),

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SET NEW ENGINES COMMANDS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    {Acc, Adv_V_Ref_New, Turn_V_Ref_New} = stability_engine:controller({Dt, Angle, Speed}, {Adv_V_Goal, Adv_V_Ref}, {Turn_V_Goal, Turn_V_Ref}),

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% DETERMINE NEW ROBOT STATE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Robot_Up_New = is_robot_up(Angle, Robot_Up),
    Next_Robot_State = get_robot_state({Robot_State, Robot_Up, Get_Up, Arm_Ready, Angle}),
    Output_Byte = get_output_state(Next_Robot_State, Angle),    

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SEND CONTROLS TO I2CBus %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    i2c_write(Acc, Turn_V_Ref_New, Output_Byte),

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% FREQUENCY STABILISATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    {N_New, Freq_New, Mean_Freq_New} = frequency_computation(Dt, N, Freq, Mean_Freq),
    smooth_frequency(T_End, T1),

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% STATE UPDATE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    T_End_New = erlang:system_time()/1.0e6,
    NewState = State#{
        robot_state => {Next_Robot_State, Robot_Up_New},
        kalman_state => {T1, X1, P1},
        old_kalman_state => {T1, Old_X1, Old_P1},
        move_speed => {Adv_V_Ref_New, Turn_V_Ref_New},
        frequency => {N_New, Freq_New, Mean_Freq_New, T_End_New},
        acc_prev => Acc
    },

    robot_loop(NewState).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% CONFIG FUNCTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

calibrate() ->
    io:format("[ROBOT] Calibrating... Do not move the pmod_nav!~n"),
    N = 500,
    Y_List = [pmod_nav:read(acc, [out_y_g]) || _ <- lists:seq(1, N)],
    Gy0 = lists:sum([Y || [Y] <- Y_List]) / N,
    io:format("[ROBOT] Done calibrating~n"),
    [grisp_led:flash(L, green, 500) || L <- [1, 2]],
    persistent_term:put(gy0, Gy0).    

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% KALMAN COMPUTATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
init_kalman() ->
    % Initiating kalman constants
    R = mat:matrix([[3.0, 0.0], [0, 3.0e-6]]), 
    Q = mat:matrix([[0.0002, 0.0], [0.0, 1.5]]),

    % Q = mat:matrix([[3.0e-5, 0.0], [0.0, 10.0]]),

    Jh = fun (_) -> mat:matrix([  	[1, 0],
								    [0, 1] ])
		 end,
    persistent_term:put(kalman_constant, {R, Q, Jh}),

    % Initial State and Covariance matrices
    X0 = mat:matrix([[0], [0]]),
    P0 = mat:matrix([[0.1, 0], [0, 0.1]]),
    {X0, P0}.

kalman_angle(Dt, Ax, Az, Gy, U, X0, P0) ->
    Gy0 = persistent_term:get(gy0),
    {R, Q, Jh} = persistent_term:get(kalman_constant),

    % Constants
    G = ?g,
    Hh = ?h + (?I / (?M * ?h)),

    % Nonlinear state model (digital twin)
    F = fun (X) ->
        Arr = mat:to_array(X),
        case Arr of
            [Th, W] ->
                Th1 = Th + W * Dt,
                W1 = W + ((G / Hh) * math:sin(Th) - (U / Hh) * math:cos(Th)) * Dt,
                mat:matrix([[Th1], [W1]]);
            _ ->
                error({unexpected_state_vector, Arr})
        end
    end,

    % Jacobian of F
    Jf = fun (X) ->
        [Th, _W] = mat:to_array(X),
        DW_dTh = ((G / Hh) * math:cos(Th) + (U / Hh) * math:sin(Th)) * Dt,
        mat:matrix([[1, Dt],
                    [DW_dTh, 1]])
    end,

    % Observation function
    H = fun (X) ->
        [Th, W] = mat:to_array(X),
        mat:matrix([[Th], [W]])
    end,

    % Measurement vector: angle from accelerometer, angular velocity from gyro
    Z = mat:matrix([[math:atan(Az / (-Ax))], [(Gy - Gy0) * ?DEG_TO_RAD]]),
    {X1, P1} = kalman:ekf({X0, P0}, {F, Jf}, {H, Jh}, Q, R, Z),

    [Th_Kalman, _W_Kalman] = mat:to_array(X1),
    Angle = Th_Kalman * ?RAD_TO_DEG,
    [Angle, {X1, P1}].

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% OLD KALMAN COMPUTATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

old_init_kalman() ->
    % Initiating kalman constants
    R = mat:matrix([[3.0, 0.0], [0, 3.0e-6]]),
    Q = mat:matrix([[3.0e-5, 0.0], [0.0, 10.0]]),
    Jh = fun (_) -> mat:matrix([  	[1, 0],
								    [0, 1] ])
		 end,
    persistent_term:put(kalman_constant, {R, Q, Jh}),

    % Initial State and Covariance matrices
    X0 = mat:matrix([[0], [0]]),
    P0 = mat:matrix([[0.1, 0], [0, 0.1]]),
    {X0, P0}.

old_kalman_angle(Dt, Ax, Az, Gy, X0, P0) ->
    Gy0 = persistent_term:get(gy0),
    {R, Q, Jh} = persistent_term:get(kalman_constant),
    
    F = fun (X) -> [Th, W] = mat:to_array(X),
				mat:matrix([ 	[Th+Dt*W],
								[W      ] ])
		end,
    Jf = fun (_) -> mat:matrix([  	[1, Dt],
								    [0, 1 ] ])
		 end,
    H = fun (X) -> [Th, W] = mat:to_array(X),
				mat:matrix([ 	[Th],
								[W ] ])
		end,
    
    Z = mat:matrix([[math:atan(Az / (-Ax))], [(Gy-Gy0)*?DEG_TO_RAD]]),
    {X1, P1} = kalman:ekf({X0, P0}, {F, Jf}, {H, Jh}, Q, R, Z),

    [Th_Kalman, _W_Kalman] = mat:to_array(X1),
    Angle = Th_Kalman * ?RAD_TO_DEG,
    [Angle, {X1, P1}].

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ROBOT STATE LOGIC %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

get_robot_state(Robot_State) -> % {Robot_state, Robot_Up, Get_Up, Arm_ready, Angle}
    case Robot_State of
        {rest, _, true, _, _} -> raising;
        {rest, _, _, _, _} -> rest;
        {raising, true, _, _, _} -> stand_up;
        {raising, _, false, _, _} -> soft_fall;
        {raising, _, _, _, _} -> raising;
        {stand_up, _, false, _, _} -> wait_for_extend;
        {stand_up, false, _, _, _} -> rest;
        {stand_up, _, _, _, _} -> stand_up;
        {wait_for_extend, _, _, _, _} -> prepare_arms;
        {prepare_arms, _, _, true, _} -> free_fall;
        {prepare_arms, _, true, _, _} -> stand_up;
        {prepare_arms, false, _, _, _} -> rest;
        {prepare_arms, _, _, _, _} -> prepare_arms;
        {free_fall, _, _, _, Angle} ->
            case abs(Angle) >10 of
                true -> wait_for_retract;
                _ ->free_fall
            end;
        {wait_for_retract, _, _, _, _} -> soft_fall;
        {soft_fall, _, _, true, _} -> rest;
        {soft_fall, _, true, _, _} -> raising;
        {soft_fall, _, _, _, _} -> soft_fall
    end.

get_output_state(State, Angle) ->
    Move_direction = get_movement_direction(Angle),    
    % Output bits = [Power, Freeze, Extend, Robot_Up_Bit, Move_direction, 0, 0, 0]
    case State of 
        rest -> 
            get_byte([0, 0, 0, 0, Move_direction, 0, 0, 0]);
        raising -> 
            get_byte([1, 0, 1, 0, Move_direction, 0, 0, 0]);
        stand_up -> 
            get_byte([1, 0, 0, 1, Move_direction, 0, 0, 0]);
        wait_for_extend -> 
            get_byte([1, 0, 1, 1, Move_direction, 0, 0, 0]);
        prepare_arms -> 
            get_byte([1, 0, 1, 1, Move_direction, 0, 0, 0]);
        free_fall -> 
            get_byte([1, 1, 1, 1, Move_direction, 0, 0, 0]);
        wait_for_retract -> 
            get_byte([1, 0, 0, 0, Move_direction, 0, 0, 0]);
        soft_fall -> 
            get_byte([1, 0, 0, 0, Move_direction, 0, 0, 0])
    end.

is_robot_up(Angle, Robot_Up) ->
    if 
        Robot_Up and (abs(Angle) > 40) ->
            false;
        not Robot_Up and (abs(Angle) < 38) -> 
            true;
        true ->
            Robot_Up
    end.

get_movement_direction(Angle) ->
    if
        Angle > 0.0 ->
            1;
        true ->
            0
    end.

get_byte(List) ->
    [A, B, C, D, E, F, G, H] = List,
    A*128 + B*64 + C*32 + D*16 + E*8 + F*4 + G*2 + H. 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% I2C COMMUNICATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

i2c_read() ->
    %Receive I2C and conversion
    I2Cbus = persistent_term:get(i2c),
    case grisp_i2c:transfer(I2Cbus, [{read, 16#40, 1, 5}]) of
        {[<<SL1,SL2,SR1,SR2,CtrlByte>>]} ->
            [Speed_L,Speed_R] = hera_com:decode_half_float([<<SL1, SL2>>, <<SR1, SR2>>]),
            Speed = (Speed_L + Speed_R)/2,
            {Speed, CtrlByte};
        {error, Reason} ->
            io:format("[ROBOT][I2C ERROR] ~p~n", [{error, Reason}]),
            {0.0, 0};
        Other ->
            io:format("[ROBOT][I2C ERROR] Unexpected response: ~p~n", [Other]),
            {0.0, 0}
    end.

i2c_write(Acc, Turn_V_Ref_New, Output_Byte) ->
    I2Cbus = persistent_term:get(i2c),
    [HF1, HF2] = hera_com:encode_half_float([Acc, Turn_V_Ref_New]),
    grisp_i2c:transfer(I2Cbus, [{write, 16#40, 1, [HF1, HF2, <<Output_Byte>>]}]).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% MISCELLANIOUS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
            N_New = 0,
            Freq_New = 0,
            Mean_Freq_New = Freq;
        true ->
            N_New = N+1,
            Freq_New = ((Freq*N)+(1/Dt))/(N+1),
            Mean_Freq_New = Mean_Freq
    end,
    {N_New, Freq_New, Mean_Freq_New}.
    
smooth_frequency(T_End, T1)->
    T2 = erlang:system_time()/1.0e6,
    Freq_Goal = persistent_term:get(freq_goal),
    Delay_Goal = 1.0/Freq_Goal * 1000.0,
    if
        T2-T_End < Delay_Goal ->
            timer:sleep(Delay_Goal-(T2-T1));
        true ->
            ok
    end.
