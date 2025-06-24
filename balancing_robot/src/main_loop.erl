-module(main_loop).

-export([robot_init/0]).

-define(RAD_TO_DEG, 180.0/math:pi()).

-define(ADV_V_MAX, 20.0).
-define(TURN_V_MAX, 80.0).
-define(KALMAN_BIAS_OFFSET, 0.6). % degrees

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% INITIALISATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

robot_init() ->

    process_flag(priority, max),

    % Start flashing LED 1 and 2 yellow until calibration and kalman inits are done
    log_buffer:add({main_loop, erlang:system_time(millisecond), calibrating}),
    [grisp_led:flash(L, yellow, 500) || L <- [1, 2]],
    calibrate(),
    {X0, P0} = kalman_computations:init_kalman(),
    {Old_X0, Old_P0} = kalman_computations:old_init_kalman(),
    log_buffer:add({main_loop, erlang:system_time(millisecond), done_calibrating}),
    [grisp_led:off(L) || L <- [1, 2]],

    %I2C bus
    I2Cbus = grisp_i2c:open(i2c1),
    persistent_term:put(i2c, I2Cbus),

    % PIDs initialization with adjusted gains
    % Pid_Speed = spawn(pid_controller, pid_init, [-0.12, -0.07, 0.0, -1, 60.0, 0.0]),
    % Pid_Stability = spawn(pid_controller, pid_init, [17.0, 0.0, 4.0, -1, -1, 0.0]),
    Pid_Speed = spawn(pid_controller, pid_init, [-0.06, -0.005, 0.0, -1, 60.0, 0.0]), 
    Pid_Stability = spawn(pid_controller, pid_init, [16.5, 0.0, 9.2, -1, -1, 0.0]), 
    persistent_term:put(controllers, {Pid_Speed, Pid_Stability}),
    persistent_term:put(freq_goal, 250.0),

    T0 = erlang:system_time()/1.0e6,
    log_buffer:add({main_loop, erlang:system_time(millisecond), robot_ready}),

    State = #{
        robot_state => {rest, false}, % {Robot_State, Robot_Up}
        kalman_state => {T0, X0, P0}, % {Tk, Xk, Pk}
        old_kalman_state => {Old_X0, Old_P0}, % {Old_Xk, Old_Pk}
        move_speed => {0.0, 0.0}, % {Adv_V_Ref, Turn_V_Ref}
        frequency => {0, 0, 200.0, T0}, % {N, Freq, Mean_Freq, T_End}
	    acc_prev => 0.0
    }, 

    robot_loop(State).

robot_loop(State) ->
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% LOGGING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    [grisp_led:color(L, aqua) || L <- [1, 2]],
    {N, Freq, Mean_Freq, T_End} = maps:get(frequency, State), 
    case N rem 50 of 0 ->
        log_buffer:add({main_loop, erlang:system_time(millisecond), robot_frequency, [Freq, N, Mean_Freq]});
    _ -> ok
    end,
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% COMPUTE Dt  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    {Tk, Xk, Pk} = maps:get(kalman_state, State),
    T1 = erlang:system_time()/1.0e6,
    Dt = (T1- Tk)/1000.0,
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% GET INPUT FROM I2CBus %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    {Speed, CtrlByte} = i2c_read(),
    [Arm_Ready, _, _, Get_Up, Forward, Backward, Left, Right] = hera_com:get_bits(CtrlByte),
    Acc_Prev = maps:get(acc_prev, State),
    Acc_SI = Acc_Prev / 100.0,  % Converti en m/s²

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% DERIVE CONTROLS FROM INPUTS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Adv_V_Goal = speed_ref(Forward, Backward),
    Turn_V_Goal = turn_ref(Left, Right),

    {Angle, X1, P1, Old_X1, Old_P1} =
    receive 
        {nav_data, [Gy_raw, Ax_raw, Az_raw]} ->
            case valid_nav_data({Gy_raw, Ax_raw, Az_raw}) of
                true ->
                    Gy = Gy_raw,
                    Ax = Ax_raw,
                    Az = Az_raw,
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% KALMAN PREDICTION + CORRECTION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    [Angle1, {X1a, P1a}] = kalman_computations:kalman_angle(Dt, Ax, Az, Gy, Acc_SI, Xk, Pk),

                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%% MEASURED DIRECT ANGLE & OLD KALMAN %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    Direct_angle = math:atan(Az / (-Ax)) * ?RAD_TO_DEG,
                    {Old_Xk, Old_Pk} = maps:get(old_kalman_state, State),
                    [Old_Angle, {Old_X1a, Old_P1a}] = kalman_computations:old_kalman_angle(Dt, Ax, Az, Gy, Old_Xk, Old_Pk),
                    case N rem 50 of 0 ->
                        log_buffer:add({main_loop, erlang:system_time(millisecond), kalman_comparison, [Angle1 - ?KALMAN_BIAS_OFFSET, Old_Angle - ?KALMAN_BIAS_OFFSET, Direct_angle - ?KALMAN_BIAS_OFFSET]});
                    _ -> ok
                    end,
                    {Angle1, X1a, P1a, Old_X1a, Old_P1a};
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% BAD NAV DATA %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                false ->
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% KALMAN PREDICTION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    {X1a, P1a} = kalman_computations:kalman_predict_only(Dt, [Xk, Pk]),
                    [Th, _] = mat:to_array(X1a),
                    {Old_Xk, Old_Pk} = maps:get(old_kalman_state, State),
                    [OldTh, _] = mat:to_array(Old_Xk),
                    case N rem 50 of 0 ->
                        log_buffer:add({main_loop, erlang:system_time(millisecond), kalman_comparison_predict_only, [Th * ?RAD_TO_DEG - ?KALMAN_BIAS_OFFSET, OldTh * ?RAD_TO_DEG - ?KALMAN_BIAS_OFFSET]});
                    _ -> ok
                    end,
                    {Th * ?RAD_TO_DEG, X1a, P1a, Old_Xk, Old_Pk}
        end
    after 0 ->
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% KALMAN PREDICTION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        {X1a, P1a} = kalman_computations:kalman_predict_only(Dt, [Xk, Pk], Acc_SI),

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% KALMAN COMPARISON %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        [Th, _] = mat:to_array(X1a),
        {Old_Xk, Old_Pk} = maps:get(old_kalman_state, State),
        [OldTh, _] = mat:to_array(Old_Xk),
        case N rem 50 of 0 ->
            log_buffer:add({main_loop, erlang:system_time(millisecond), kalman_comparison_predict_only, [Th * ?RAD_TO_DEG - ?KALMAN_BIAS_OFFSET, OldTh * ?RAD_TO_DEG - ?KALMAN_BIAS_OFFSET]});
        _ -> ok
        end,
        
        {Th * ?RAD_TO_DEG, X1a, P1a, Old_Xk, Old_Pk}
    end,
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ANGLE CORRECTION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Apply a bias offset to the angle to correct for the robot's physical characteristics
    Angle_Corrected = Angle - ?KALMAN_BIAS_OFFSET,

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SONAR MEASURE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Sonar_Data =
    receive
        {sonar_data, robot_main, [D]} ->
            % Convert distance from cm to m
            D_M = D / 100.0,
            case N rem 50 of 0 ->
                log_buffer:add({main_loop, erlang:system_time(millisecond), sonar_measure, [robot_main, D_M]});
            _ -> ok
            end,    
            D_M;
        {sonar_data, robot_front_left, [D]} ->
            % Convert distance from cm to m
            D_M = D / 100.0,
            case N rem 50 of 0 ->
                log_buffer:add({main_loop, erlang:system_time(millisecond), sonar_measure, [robot_front_left, D_M]});
            _ -> ok
            end,
            D_M;
        {sonar_data, robot_front_rigth, [D]} ->
            % Convert distance from cm to m
            D_M = D / 100.0,
            case N rem 50 of 0 ->
                log_buffer:add({main_loop, erlang:system_time(millisecond), sonar_measure, [robot_front_rigth, D_M]});
            _ -> ok
            end,
            D_M
    after 0 ->
        % If no sonar data received, use a default value (e.g., 0.0)
        0.0
    end,

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SET NEW ENGINES COMMANDS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    {Adv_V_Ref, Turn_V_Ref} = maps:get(move_speed, State),
    {Acc, Adv_V_Ref_New, Turn_V_Ref_New} = stability_engine:controller({Dt, Angle_Corrected, Speed, Sonar_Data}, {Adv_V_Goal, Adv_V_Ref}, {Turn_V_Goal, Turn_V_Ref}, N),

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% DETERMINE NEW ROBOT STATE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    {Robot_State, Robot_Up} = maps:get(robot_state, State),
    Robot_Up_New = is_robot_up(Angle_Corrected, Robot_Up),
    Next_Robot_State = get_robot_state({Robot_State, Robot_Up, Get_Up, Arm_Ready, Angle_Corrected}),
    Output_Byte = get_output_state(Next_Robot_State, Angle_Corrected),

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SEND CONTROLS TO I2CBus %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    i2c_write(Acc, Turn_V_Ref_New, Output_Byte),

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% FREQUENCY STABILISATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    {N_New, Freq_New, Mean_Freq_New} = frequency_computation(Dt, N, Freq, Mean_Freq),
    %   Imposed maximum frequency
    T2 = erlang:system_time()/1.0e6,
    Freq_Goal = persistent_term:get(freq_goal),
    Delay_Goal = 1.0/Freq_Goal * 1000.0,
    if
        T2-T_End < Delay_Goal ->
            wait(Delay_Goal-(T2-T1));
        true ->
            ok
    end,

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% STATE UPDATE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    T_End_New = erlang:system_time()/1.0e6,
    NewState = State#{
        robot_state => {Next_Robot_State, Robot_Up_New},
        old_kalman_state => {Old_X1, Old_P1},
        kalman_state => {T1, X1, P1},
        move_speed => {Adv_V_Ref_New, Turn_V_Ref_New},
        frequency => {N_New, Freq_New, Mean_Freq_New, T_End_New},
        acc_prev => Acc
    },
    robot_loop(NewState).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% CONFIG FUNCTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
calibrate() ->
    N = 500,
    Y_List = [pmod_nav:read(acc, [out_y_g]) || _ <- lists:seq(1, N)],
    Gy0 = lists:sum([Y || [Y] <- Y_List]) / N,
    persistent_term:put(gy0, Gy0).

valid_nav_data({Gy, Ax, Az}) ->
    Gy >= -250.0 andalso Gy =< 250.0 andalso
    Ax >= -12.0 andalso Ax =< 12.0 andalso
    Az >= -12.0 andalso Az =< 12.0.

wait(T) -> 
	Tnow = erlang:system_time()/1.0e6,
	wait_help(Tnow,Tnow+T).
wait_help(Tnow, Tend) when Tnow >= Tend -> ok;
wait_help(_, Tend) -> 
    Tnow = erlang:system_time()/1.0e6,
    wait_help(Tnow,Tend).
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
    
smooth_frequency(T_End, T1) ->
    Freq_Goal = persistent_term:get(freq_goal),
    Delay_Goal = 1000.0 / Freq_Goal,
    Remaining = Delay_Goal - (T1 - T_End),
    SleepTime = trunc(max(Remaining, 0)),
    if SleepTime > 0 -> timer:sleep(SleepTime); true -> ok end.

