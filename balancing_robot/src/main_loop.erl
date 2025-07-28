-module(main_loop).

-export([robot_init/0]).

-define(RAD_TO_DEG, 180.0/math:pi()).

-define(ADV_V_MAX, 20.0).
-define(TURN_V_MAX, 80.0).
-define(LOG_INTERVAL, 500). % ms

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% INITIALISATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

robot_init() ->
    process_flag(priority, max),
    log_buffer:add({main_loop, erlang:system_time(millisecond), calibrating}),
    calibrate(),
    {X0, P0} = kalman_computations:init_kalman(),
    log_buffer:add({main_loop, erlang:system_time(millisecond), done_calibrating}),

    %I2C bus
    I2Cbus = grisp_i2c:open(i2c1),
    persistent_term:put(i2c, I2Cbus),

    % PIDs initialization with adjusted gains
    Pid_Speed = spawn(hera_pid_controller, pid_init, [-0.071, -0.053, 0.0, -1, 15.0, 0.0]), 
    Pid_Stability = spawn(hera_pid_controller, pid_init, [16.3, 0.0, 9.4, -1, -1, 0.0]), 
    persistent_term:put(controllers, {Pid_Speed, Pid_Stability}),
    persistent_term:put(freq_goal, 250.0),

    T0 = erlang:system_time()/1.0e6,
    log_buffer:add({main_loop, erlang:system_time(millisecond), robot_ready}),

    State = #{
        robot_state => {rest, false}, % {Robot_State, Robot_Up}
        kalman_state => {T0, X0, P0}, % {Tk, Xk, Pk}
        move_speed => {0.0, 0.0}, % {Adv_V_Ref, Turn_V_Ref}
        frequency => {0, 0, 200.0, T0}, % {N, Freq, Mean_Freq, T_End}
        acc_prev => 0.0, % Acc_Prev
        sonar => {robot_front_left, 0.0, 0, none}, % {Sonar_Role, T_End_Sonar, Current_Seq, Prev_Dist}
        last_log_time => erlang:system_time(millisecond) % LastLog
    },

    robot_loop(State).

robot_loop(State) ->
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Map Access %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    {N, Freq, Mean_Freq, T_End} = maps:get(frequency, State),
    {Tk, Xk, Pk} = maps:get(kalman_state, State),

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Dt Computation %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    T1 = erlang:system_time()/1.0e6,
    Dt = (T1- Tk)/1000.0,

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% LOGGING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    LastLog = maps:get(last_log_time, State),
    {Robot_State, Robot_Up} = maps:get(robot_state, State),
    {DoLog, New_LastLog}= logging(T1, LastLog),
    send_to_server(Robot_State),
    add_log({main_loop, erlang:system_time(millisecond), robot_frequency, [Freq, N, Mean_Freq]}, DoLog),

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% INPUT FROM I2CBus & CONTROLS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    {Speed, CtrlByte} = i2c_read(),
    [Arm_Ready, _, _, Get_Up, Forward, Backward, Left, Right] = hera_com:get_bits(CtrlByte),
    Adv_V_Goal = speed_ref(Forward, Backward),
    Turn_V_Goal = turn_ref(Left, Right),

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SONAR LOGIC %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    {Sonar_Role, T_End_Sonar, Current_Seq, Prev_Dist} = maps:get(sonar, State),
    Sonar_Clock_Now = erlang:monotonic_time(second) + erlang:monotonic_time(microsecond) / 1.0e6,
    {Sonar_Data, New_Seq} = sonar_message_handling(Current_Seq, Prev_Dist),

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% KALMAN LOGIC %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Acc_Prev = maps:get(acc_prev, State),
    {Angle, X1, P1} = kalman_message_handling(Xk, Pk, Acc_Prev, Dt, DoLog),
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SET NEW ENGINES COMMANDS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    {Adv_V_Ref, Turn_V_Ref} = maps:get(move_speed, State),
    {Acc, Adv_V_Ref_New, Turn_V_Ref_New} = stability_engine:controller({Dt, Angle, Speed, Sonar_Data}, {Adv_V_Goal, Adv_V_Ref}, {Turn_V_Goal, Turn_V_Ref}, DoLog),

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% NEW ROBOT STATE  & I2CBus WRITE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Robot_Up_New = is_robot_up(Angle, Robot_Up),
    Next_Robot_State = get_robot_state({Robot_State, Robot_Up, Get_Up, Arm_Ready, Angle}),
    Output_Byte = get_output_state(Next_Robot_State),
    i2c_write(Acc, Turn_V_Ref_New, Output_Byte),

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SONAR REQUEST %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Sonar_Turn_Next = sonar_request(Sonar_Role, T_End_Sonar, Sonar_Clock_Now, Adv_V_Goal),
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% FREQUENCY STABILISATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    {N_New, Freq_New, Mean_Freq_New} = frequency_computation(Dt, N, Freq, Mean_Freq),
    maximum_frequency(T1, T_End),

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% STATE UPDATE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    T_End_New = erlang:system_time()/1.0e6,
    NewState = State#{
        robot_state => {Next_Robot_State, Robot_Up_New},
        kalman_state => {T1, X1, P1},
        move_speed => {Adv_V_Ref_New, Turn_V_Ref_New},
        frequency => {N_New, Freq_New, Mean_Freq_New, T_End_New},
        acc_prev => Acc,
        sonar => {Sonar_Turn_Next, Sonar_Clock_Now, New_Seq, Sonar_Data}, 
        last_log_time => New_LastLog
    },
    robot_loop(NewState).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ROBOT STATE LOGIC %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

get_robot_state(Robot_State) -> % {Robot_state, Robot_Up, Get_Up, Arm_ready, Angle} = {Robot_state, stabilité, Get_statique, Arm_ready, Angle}
    case Robot_State of
        {rest, true, _, _, _} -> dynamic;
        {rest, _, _, _, _} -> rest;
        {dynamic, _, true, _, _} -> static;
        {dynamic, false, _, _, _} -> rest;
        {dynamic, _, _, _, _} -> dynamic;
        {static, _, false, _, _} -> dynamic;
        {static, false, _, _, _} -> rest;
        {static, _, _, _, _} -> static
    end.

get_output_state(State) ->   
    % Output bits = [Power, Freeze, Extend, Robot_Up_Bit, Move_direction, 0, 0, 0]
    case State of 
        rest      -> get_byte([0, 0, 0, 0, 0, 0, 0, 0]);
        dynamic   -> get_byte([1, 0, 0, 1, 0, 0, 0, 0]);
        static    -> get_byte([1, 1, 1, 1, 0, 0, 0, 0])
    end.

is_robot_up(Angle, Robot_Up) ->
    if 
        Robot_Up and (abs(Angle) > 80) ->
            false;
        not Robot_Up and (abs(Angle) < 78) -> 
            true;
        true ->
            Robot_Up
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
        [<<SL1,SL2,SR1,SR2,CtrlByte>>] ->
            [Speed_L,Speed_R] = hera_com:decode_half_float([<<SL1, SL2>>, <<SR1, SR2>>]),
            Speed = (Speed_L + Speed_R)/2,
            {Speed, CtrlByte};
        {error, Reason} ->
            io:format("[ROBOT][I2C ERROR] Error response: ~p~n", [{error, Reason}]),
            timer:sleep(5000),
            i2c_read();
        Other ->
            io:format("[ROBOT][I2C ERROR] Unexpected response: ~p~n", [Other]),
            timer:sleep(5000),
            i2c_read()
    end.

i2c_write(Acc, Turn_V_Ref_New, Output_Byte) ->
    I2Cbus = persistent_term:get(i2c),
    case hera_com:encode_half_float([Acc, Turn_V_Ref_New]) of
        [HF1, HF2] ->
            grisp_i2c:transfer(I2Cbus, [{write, 16#40, 1, [HF1, HF2, <<Output_Byte>>]}]);
        Error ->
            log_buffer:add({i2c_error, erlang:system_time(millisecond), Error})
    end.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% MISCELLANIOUS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
speed_ref(Forward, Backward) ->
    if
        Forward ->
            Adv_V_Goal = -?ADV_V_MAX;
        Backward ->
            Adv_V_Goal = +?ADV_V_MAX;
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

maximum_frequency(T1, T_End) ->
    T2 = erlang:system_time()/1.0e6,
    Freq_Goal = persistent_term:get(freq_goal),
    Delay_Goal = 1.0/Freq_Goal * 1000.0,
    if
        T2-T_End < Delay_Goal ->
            wait(Delay_Goal-(T2-T1));
        true ->
            ok
    end.

wait(T) -> 
	Tnow = erlang:system_time()/1.0e6,
	wait_help(Tnow,Tnow+T).
wait_help(Tnow, Tend) when Tnow >= Tend -> ok;
wait_help(_, Tend) -> 
    Tnow = erlang:system_time()/1.0e6,
    wait_help(Tnow,Tend).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% LOGGING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

logging(Now, Last_Log_Time) ->
    case Now - Last_Log_Time > ?LOG_INTERVAL of
        true ->
            {true, Now};
        false ->
            {false, Last_Log_Time}
    end.

add_log(Log, DoLog) ->
    case DoLog of
        true ->
            log_buffer:add(Log);
        false -> ok
    end.

send_to_server(Robot_State) ->
    case Robot_State of 
        static ->
            case persistent_term:get(server_on) of
                true ->
                    log_buffer:flush_to_server(server, persistent_term:get(name));
                _ ->
                    ok
            end;
        _ -> ok
    end.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SONAR %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

sonar_request(Sonar_Role, T_End_Sonar, Sonar_Clock_Now, Adv_V_Goal) ->
    case (Sonar_Clock_Now - T_End_Sonar > 0.05) of
            true ->
                case Adv_V_Goal of
                    V when V < 0 ->
                        % Recule : on interroge uniquement le sonar arrière
                        persistent_term:get(pid_sonar) ! {authorize, self()},
                        Sonar_Role;
                    V when V > 0 ->
                        % Avance : alternance front_left <-> front_right
                        Next_Role = case Sonar_Role of
                            robot_front_left -> robot_front_right;
                            robot_front_right -> robot_front_left
                        end,
                        hera_com:send_unicast(Next_Role, "authorize", "UTF8"),
                        Next_Role;
                    _ ->
                        % À l’arrêt : ne change pas
                        Sonar_Role
                end;
            _ -> Sonar_Role
        end.

sonar_message_handling(Current_Seq, Prev_Dist) ->
    receive
        {sonar_data, Sonar_Name, [D, Seq]} ->
            D_M = D / 100.0,
            add_log({main_loop, erlang:system_time(millisecond), new_sonar_measure, [Sonar_Name, D_M]}, true),
            io:format("[ROBOT][SONAR] Sonar data received: ~p from ~p~n", [D_M, Sonar_Name]),
            {D_M, Seq}
    after 0 ->
        {Prev_Dist, Current_Seq}
    end.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% KALMAN  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
kalman_message_handling(Xk, Pk, Acc_Prev, Dt, DoLog) ->
    Acc_SI = Acc_Prev / 100.0, % Convert acceleration from cm/s^2 to m/s^2
    {Angle, X1, P1} = 
    receive 
        {nav_data, [Gy, Ax, Az]} ->
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% KALMAN PREDICTION + CORRECTION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            [Angle1, {X1a, P1a}] = kalman_computations:kalman_angle(Dt, Ax, Az, Gy, Acc_SI, Xk, Pk),

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%% MEASURED DIRECT ANGLE  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            Direct_angle = math:atan(Az / (-Ax)) * ?RAD_TO_DEG,
            add_log({main_loop, erlang:system_time(millisecond), kalman_comparison, [Angle1, Direct_angle]}, DoLog),
                {Angle1, X1a, P1a}
    after 0 ->
        kalman_predict_only(Xk, Pk, Dt, DoLog, Acc_SI) 
    end,
    {Angle, X1, P1}.


kalman_predict_only(Xk, Pk, Dt, DoLog, Acc_SI) ->
    % Kalman prediction
    [Angle1, {X1a, P1a}] = kalman_computations:kalman_predict_only(Dt, [Xk, Pk], Acc_SI),
    add_log({main_loop, erlang:system_time(millisecond), kalman_comparison_predict_only, [Angle1]}, DoLog),
    {Angle1, X1a, P1a}.

calibrate() ->
    N = 500,
    Y_List = [pmod_nav:read(acc, [out_y_g]) || _ <- lists:seq(1, N)],
    Gy0 = lists:sum([Y || [Y] <- Y_List]) / N,
    persistent_term:put(gy0, Gy0).

