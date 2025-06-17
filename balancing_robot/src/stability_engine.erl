-module(stability_engine).

-export([controller/4]).

-define(ADV_V_MAX, 30.0).
-define(ADV_ACCEL, 75.0).

-define(TURN_V_MAX, 80.0).
-define(TURN_ACCEL, 400.0).

controller({Dt, Angle, Speed}, {Adv_V_Goal, Adv_V_Ref}, {Turn_V_Goal, Turn_V_Ref}, N) ->
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% CONTROLLER %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    {Pid_Speed, Pid_Stability} = persistent_term:get(controllers),

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ACCELERATION SATURATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Adv_V_Ref_New = saturate_acceleration(Adv_V_Goal, Adv_V_Ref, Dt, ?ADV_ACCEL, ?ADV_V_MAX),
    Turn_V_Ref_New = saturate_acceleration(Turn_V_Goal, Turn_V_Ref, Dt, ?TURN_ACCEL, ?TURN_V_MAX),

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ADVANCED SPEED CONTROLLER %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Pid_Speed ! {self(), {set_point, Adv_V_Ref_New}},
    Pid_Speed ! {self(), {input, Speed}},
    receive {_, {control, Target_angle}} -> ok end, 

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% COMPUTE ANGLE CORRECTION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Adjust the target angle based on the speed: the robot should lean in the direction it is moving forward or backward
    % But the acceleration of the top and bottom of a rectangular paralelepiped robot is not the same, 
    % so we need to adjust the angle accordingly.
    % Acc_Estim = (Speed - maps:get(prev_speed, State)) / Dt,
    % Ka = 0.1, % Approximation of a pendulum's gain, can be tuned
    % Target_angle_Adv = Target_angle + Ka * Acc_Estim,

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% STABILITY CONTROLLER %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Pid_Stability ! {self(), {set_point, Target_angle}},
    Pid_Stability ! {self(), {input, Angle}},
    receive {_, {control, Acc}} -> ok end,

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% LOGGING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    case N rem 50 of 0 ->
            log_buffer:add({controller, erlang:system_time(millisecond), speed_controller, [Adv_V_Ref_New, Speed, Target_angle]}),
            log_buffer:add({controller, erlang:system_time(millisecond), stability_controller, [Target_angle, Angle, Acc]});
    _ -> ok
    end,
    
    {Acc, Adv_V_Ref_New, Turn_V_Ref_New}.

% Saturates the acceleration based on the goal and reference speed.
% If the goal is positive, it accelerates towards the goal, if negative, it decelerates.
% If the goal is zero, it checks the reference speed and applies a deceleration or acceleration based on the current speed.
saturate_acceleration(Goal, Ref, Dt, Accel, V_Max) ->
    case Goal of
        G when G > 0.0 ->
            pid_controller:saturation(Ref + Accel * Dt, V_Max);
        G when G < 0.0 ->
            pid_controller:saturation(Ref - Accel * Dt, V_Max);
        _ ->
            case Ref of
                R when R > 0.5 ->
                    pid_controller:saturation(Ref - Accel * Dt, V_Max);
                R when R < -0.5 ->
                    pid_controller:saturation(Ref + Accel * Dt, V_Max);
                _ ->
                    0.0
            end
    end.
