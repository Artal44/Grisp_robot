
-module(stability_engine).

-export([controller/5]).

-define(ADV_V_MAX, 30.0).
-define(ADV_ACCEL, 75.0).

-define(TURN_V_MAX, 80.0).
-define(TURN_ACCEL, 400.0).

%% @doc The `controller/5` function processes the control logic for the balancing robot.
%% @param {Dt, Angle, Speed} - A tuple containing the time delta (Dt), the current angle of the robot (Angle), and the current speed of the robot (Speed).
%% @param {Pid_Speed, Pid_Stability, Pid_Obstacle_Avoidance} - A tuple containing the PID controller parameters for speed, stability and obstacle avoidance.
%% @param {Adv_V_Goal, Adv_V_Ref} - A tuple containing the goal and reference values for the advance velocity.
%% @param {Turn_V_Goal, Turn_V_Ref} - A tuple containing the goal and reference values for the turning velocity.
%% @param {D_Kalman, Interpolation_Type} - A tuple containing the distance to the closest object and the type of interpolation used for deceleration.
%% @return - The function returns a tuple with the updated control values for the robot.
controller({Dt, Angle, Speed}, {Pid_Speed, Pid_Stability, Pid_Obstacle_Avoidance}, {Adv_V_Goal, Adv_V_Ref}, {Turn_V_Goal, Turn_V_Ref}, {D_Kalman, Interpolation_Type}) ->
    
    % ==========================
    % Obstacle avoidance
    % ==========================

    % Compute the Advance Velocity Goal based on the distance to the closest object
    Adv_V_Goal_Adapted = obstacle_avoidance:compute_target_speed(Adv_V_Goal, D_Kalman, Interpolation_Type),
    
    % Update Obstacle Avoidance PID
    Obstacle_Adjustment = run_pid(Pid_Obstacle_Avoidance, Adv_V_Goal_Adapted, Speed),

    % Compute the new Advance Velocity Reference
    Adv_V_Ref_Adjusted = Adv_V_Ref + Obstacle_Adjustment,

    % ==========================
    % Saturate Advance and Turn Velocities
    % ==========================

    % Compute the new Advance Velocity Reference
    Adv_V_Ref_New = saturation_velocity(Adv_V_Goal, Adv_V_Ref_Adjusted, Dt, ?ADV_ACCEL, ?ADV_V_MAX),
    Turn_V_Ref_New = saturation_velocity(Turn_V_Goal, Turn_V_Ref, Dt, ?TURN_ACCEL, ?TURN_V_MAX),

    % ==========================
    % Speed PID Control
    % ==========================

    % Compute the target angle based on the Advance Velocity Reference
    Target_Angle = run_pid(Pid_Speed, Adv_V_Ref_New, Speed),

    % ==========================
    % Stability PID Control
    % ==========================

    % Compute the new Advance Velocity Reference based on the target angle
    Acc = run_pid(Pid_Stability, Target_Angle, Angle),

    % ==========================
    % Return outputs
    % ==========================

    {Acc, Adv_V_Ref_New, Turn_V_Ref_New}.


%% @doc The function computes the new reference velocity based on the goal velocity, reference velocity, time delta, acceleration constant, and maximum velocity.
%% It applies a saturation effect to ensure the new reference velocity does not exceed the maximum velocity.
%% @param {V_Goal} - The goal velocity.
%% @param {V_Ref} - The reference velocity.
%% @param {Dt} - The time delta.
%% @param {Accel_Const} - The acceleration constant.
%% @param {V_Max} - The maximum velocity.
%% @return - The new reference velocity after applying the saturation effect.
saturation_velocity(V_Goal, V_Ref, Dt, Accel_Const, V_Max) ->
    if
        V_Goal > 0.0 ->
            % Goal positive: accelerate positively
            New_V_Ref = pid_controller:saturation(V_Ref + Accel_Const * Dt, V_Max);
        V_Goal < 0.0 ->
            % Goal negative: accelerate negatively
            New_V_Ref = pid_controller:saturation(V_Ref - Accel_Const * Dt, V_Max);
        true ->
            % Goal zero: brake toward 0
            if
                V_Ref > 0.5 ->
                    New_V_Ref = pid_controller:saturation(V_Ref - Accel_Const * Dt, V_Max);
                V_Ref < -0.5 ->
                    New_V_Ref = pid_controller:saturation(V_Ref + Accel_Const * Dt, V_Max);
                true ->
                    New_V_Ref = 0.0
            end
    end,
    New_V_Ref.
    
%% @doc Helper function to run a PID loop.
%% It sends a message to the PID process with the set point and input value,
%% @param {Pid} - The PID process identifier.
%% @param {Set_Point} - The set point for the PID controller.
%% @param {Input} - The input value for the PID controller.
%% @return - The output value from the PID controller.
run_pid(Pid, Set_Point, Input) ->
    Pid ! {self(), {set_point, Set_Point}},
    Pid ! {self(), {input, Input}},
    receive {_, {control, Output_Value}} -> ok end,
    Output_Value.
