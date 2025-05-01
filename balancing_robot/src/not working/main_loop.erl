-module(main_loop).

-behavior(application).

-define(RAD_TO_DEG, 180.0/math:pi()).
-define(LOG_DURATION, 15000).
-define(FREQ_GOAL, 300.0).

-export([start/2, stop/1]).
-export([robot_init/0]).

%============================================================================================================================================
%========================================================= BASIC GRISP FUNC =================================================================
%============================================================================================================================================

% Application start callback
start(_Type, _Args) ->
    io:format("[ROBOT] Starting ...~n"),
    {ok, Supervisor} = balancing_robot_sup:start_link(), % Start the supervisor
    [grisp_led:flash(L, yellow, 500) || L <- [1, 2]],

    config(),
    {ok, Supervisor}. % Return ok and the supervisor

% Application stop callback
stop(_State) -> ok.

%============================================================================================================================================
%========================================================= CONFIG TIME FUNC =================================================================
%============================================================================================================================================

config() ->
    %who_am_i(),
    
    numerl:init(), % Initialize numerl library
    io:format("[ROBOT] Adding devices to Grisp Card...~n"),
    _ = add_device(spi2, pmod_nav),
    _ = add_device(uart, pmod_maxsonar), % Add PMOD Sonar to Grisp Card on port UART
    pmod_nav:config(acc, #{odr_g => {hz,238}}), % Configure PMOD Nav accelerometer with output data rate of 238 Hz

    % Open I2C bus for communication with external devices (ESP32, etc.)
    io:format("[ROBOT] Opening I2C bus...~n"),
    open_i2c(),

    hera_subscribe:subscribe(self()),
    % Initialize measurements and data storage
    io:format("[ROBOT] Initializing Hera measurements...~n"),
    {ok, Gyro_Pid} = start_hera_measure(gyroscope_measure), % Start gyroscope measurements
    io:format("[ROBOT] Gyroscope PID: ~p~n", [Gyro_Pid]),
    timer:sleep(2000),
    {ok, Sonar_Pid} = start_hera_measure(sonar_measure), % Start sonar measurements
    io:format("[ROBOT] Sonar PID: ~p~n", [Sonar_Pid]),
    timer:sleep(2000),
    {ok, Kal_Pid} = start_hera_measure(kalman_stability), % Start kalman measurements
    io:format("[ROBOT] Kalman PID: ~p~n", [Kal_Pid]),

    persistent_term:put(hera_gyro, Gyro_Pid),
    persistent_term:put(hera_kal, Kal_Pid),
    persistent_term:put(hera_sonar, Sonar_Pid),
    persistent_term:put(robot_main, self()),
    timer:sleep(1000),
    robot_init(),
    ok.

start_hera_measure(Measure) ->
    % Start the specified Hera measurement
    try
        hera:start_measure(Measure, [])
    catch
        _:_ -> 
            % Try again
            io:format("[ROBOT] Error starting measure ~p. Retrying after 10s...~n", [Measure]),
            timer:sleep(10000),
            start_hera_measure(Measure)
    end.

add_device(Port, Device) ->
    % Add a device to the Grisp card
    try
        io:format("[ROBOT] Adding device ~p to port ~p...~n", [Device, Port]),
        grisp:add_device(Port, Device)
    catch
        _:_ ->
            io:format("[ROBOT] Error adding device ~p. Retrying after 10s...~n", [Device]),
            timer:sleep(10000),
            add_device(Port, Device)
    end.

open_i2c() ->
    try
        I2Cbus = grisp_i2c:open(i2c1),
        persistent_term:put(i2c, I2Cbus),
        io:format("[ROBOT] I2C bus opened successfully.~n")
    catch
        _:_ ->
            io:format("[ROBOT] Error opening I2C bus. Retrying after 10 seconds...~n"),
            timer:sleep(10000),
            open_i2c()
    end.

%============================================================================================================================================
%========================================================= LOOP =============================================================================
%============================================================================================================================================

robot_init() ->
    % Set maximum process priority for real-time critical tasks
    %process_flag(priority, max),

    % get role of the robot
    Role = role_of_robot(), 

    % Initialize PID controllers for speed obstacle avoidance and stability
    Pid_Speed = spawn(pid_controller, pid_init, [-0.12, -0.07, 0.0, -1, 60.0, 0.0]),
    persistent_term:put(pid_speed, Pid_Speed),
    Pid_Stability = spawn(pid_controller, pid_init, [17.0, 0.0, 4.0, -1, -1, 0.0]),
    persistent_term:put(pid_stability, Pid_Stability),
    Pid_Obstacle_Avoidance = spawn(pid_controller, pid_init, [-0.06, -0.02, 0.0, -1, 60.0, 0.0]),
    persistent_term:put(pid_obstacle_avoidance, Pid_Obstacle_Avoidance),
    io:format("[ROBOT] Pid of the speed controller: ~p.~n", [Pid_Speed]),
    io:format("[ROBOT] Pid of the stability controller: ~p.~n", [Pid_Stability]),
    io:format("[ROBOT] Pid of the obstacle avoidance controller: ~p~n", [Pid_Obstacle_Avoidance]),
    io:format("[ROBOT] Starting movement of the robot.~n"),    
    T0 = erlang:system_time()/1.0e6,   

    % State definition
    State = #{
    role => Role,
    robot_state => {rest, false},
    logging_state => {0, T0, []},
    movement_controls => {0.0, 0.0}
    },

    robot_main(State).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Main Robot Control Loop
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% @doc Main control loop for the robot.
%% This function handles sensor data acquisition, control logic, and communication with the ESP32.
%% @param Robot_State - The current state of the robot.
%% @return - The updated state of the robot after processing the loop.
robot_main(State) ->
    
% Accessing values from the dictionary
    Role = maps:get(role, State),
    {Adv_V_Ref, Turn_V_Ref} = maps:get(movement_controls, State),
    {Robot_state, Robot_Up} = maps:get(robot_state, State),
    {Logging, Log_End, Log_List} = maps:get(logging_state, State),    

    % Select between kalman and comp filter
    %[{_, _, _, [Tk, Xk, _]}] = hera_data:get(k_stability_state),

    % Print role of the robot
    io:format("[ROBOT] Role of the robot: ~p.~n", [Role]),
    
    receive
        {kalman_value, [_, Xk, _, Dt]} ->
                {Speed, Arm_Ready, _, _, Get_Up, Adv_V_Goal, Turn_V_Goal} = i2C_data(),
                %[{_, _, _, [_, Angle_Complem, _]}] = hera_data:get(comp_filter_state),
                [Th_Kalman, _W_Kalman, D_Kalman] = mat:to_array(Xk),
                Angle_Kalman = Th_Kalman * ?RAD_TO_DEG,
                Angle = Angle_Kalman,

                Pid_Obstacle_Avoidance = persistent_term:get(pid_obstacle_avoidance),
                Pid_Speed = persistent_term:get(pid_speed),
                Pid_Stability = persistent_term:get(pid_stability),
                %[{_, _, _, [Dt]}] = hera_data:get(delta_time),

                {Acc, Adv_V_Ref_New, Turn_V_Ref_New} = stability_engine:controller(
                    {Dt, Angle, Speed},
                    {Pid_Speed, Pid_Stability, Pid_Obstacle_Avoidance},
                    {Adv_V_Goal, Adv_V_Ref},
                    {Turn_V_Goal, Turn_V_Ref},
                    {D_Kalman, exponential}
                ),

                Robot_is_Up = is_robot_standing(Angle, Robot_Up),
                
                New_robot_state = get_robot_state({Robot_state, Robot_Up, Get_Up, Arm_Ready, Angle}),

                Move_direction = get_movement_direction(Angle),    
                Output_state = get_output_state(New_robot_state, Move_direction),
                Output_Byte = get_byte(Output_state),


                [HF1, HF2] = hera_com:encode_half_float([Acc, Turn_V_Ref_New]),
                I2Cbus = persistent_term:get(i2c),
                grisp_i2c:transfer(I2Cbus, [{write, 16#40, 1, [HF1, HF2, <<Output_Byte>>]}]),

                %{_, T_End} = update_frequency(Dt),

                %stabilize_frequency(T_End, Tk),

                New_State = State#{
                    role => Role,
                    robot_state => {New_robot_state, Robot_is_Up},
                    logging_state => {Logging, Log_End, Log_List},
                    movement_controls => {Adv_V_Ref_New, Turn_V_Ref_New}},
            
                robot_main(New_State);
        {sonar_data, From, [Sonar_Distance]} ->
            % Handle sonar data
            io:format("[ROBOT] Sonar distance from ~p : ~p cm~n", [From, Sonar_Distance])
    end.

   
%============================================================================================================================================
%========================================================= HELPER FUNCTION ==================================================================
%============================================================================================================================================
    
i2C_data() ->
    I2Cbus = persistent_term:get(i2c),
    % Receive data from the ESP32 via I2C and decode it
    [<<SL1,SL2,SR1,SR2,CtrlByte>>] = grisp_i2c:transfer(I2Cbus, [{read, 16#40, 1, 5}]),
    
    % Decode the half-float values for left and right wheel speeds
    [Speed_L, Speed_R] = hera_com:decode_half_float([<<SL1, SL2>>, <<SR1, SR2>>]),
    
    % Compute the average speed of the robot based on the left and right wheel speeds
    Speed = (Speed_L + Speed_R) / 2,

    [Arm_Ready, Switch, Test, Get_Up, Forward, Backward, Left, Right] = hera_com:get_bits(CtrlByte), % Get General command from LilyGo

    Adv_V_Goal = helper_module:speed_ref(Forward, Backward), % Get movement goal

    Turn_V_Goal = helper_module:turn_ref(Left, Right), % Get rotation goal
    {Speed, Arm_Ready, Switch, Test, Get_Up, Adv_V_Goal, Turn_V_Goal}.

is_robot_standing(Angle, Robot_Up) ->
    if 
        Robot_Up and (abs(Angle) > 20) ->
            false;
        not Robot_Up and (abs(Angle) < 18) -> 
            true;
        true ->
            Robot_Up
    end.

get_movement_direction(Angle) ->
    case Angle > 0.0 of
        true ->
            1;
        false ->
            0
    end.

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

get_output_state(State, Move_direction) ->
    case State of
        rest -> 
            [0, 0, 0, 0, Move_direction, 0, 0, 0];
        raising -> 
            [1, 0, 1, 0, Move_direction, 0, 0, 0];
        stand_up -> 
            [1, 0, 0, 1, Move_direction, 0, 0, 0];
        wait_for_extend -> 
            [1, 0, 1, 1, Move_direction, 0, 0, 0];
        prepare_arms -> 
            [1, 0, 1, 1, Move_direction, 0, 0, 0];
        free_fall -> 
            [1, 1, 1, 1, Move_direction, 0, 0, 0];
        wait_for_retract -> 
            [1, 0, 0, 0, Move_direction, 0, 0, 0];
        soft_fall -> 
            [1, 0, 0, 0, Move_direction, 0, 0, 0]
    end.

get_byte(List) ->
    [A, B, C, D, E, F, G, H] = List,
    A*128 + B*64 + C*32 + D*16 + E*8 + F*4 + G*2 + H. 

update_frequency(Dt)->    
    T_End_New = erlang:system_time()/1.0e6,
    [{ _, _, Seq, [N, Freq, Mean_Freq, T_End]}] = hera_data:get(frequency),
    {N_New, Freq_New, Mean_Freq_New} = helper_module:frequency_computation(Dt, N, Freq, Mean_Freq),
    hera_data:store(frequency, node(), Seq+1, [N_New, Freq_New, Mean_Freq_New, T_End_New]),
    io:format("[ROBOT] Freq : ~p~n",[Mean_Freq_New]),
    {N, T_End}.

stabilize_frequency(T_End, Tk) ->
    T2 = erlang:system_time()/1.0e6,
    Delay_Goal = 1.0 / ?FREQ_GOAL * 1000.0,

    if
        T2 - T_End < Delay_Goal ->
            helper_module:wait(Delay_Goal - (T2 - Tk));
        true ->
            ok
    end.

role_of_robot() ->
    {ok, Grisp_Id} =  helper_module:get_grisp_id(),

    % Print the role of the robot to the console
    io:format("[ROBOT] ID of the robot: ~p.~n", [Grisp_Id]),

    case Grisp_Id of
        0 -> main;
        1 -> front_left;
        2 -> front_right;
        _ -> unknown_role
    end.