-module(stability_engine).

-export([controller/3]).

-define(ADV_V_MAX, 30.0).
-define(ADV_ACCEL, 75.0).

-define(TURN_V_MAX, 80.0).
-define(TURN_ACCEL, 400.0).

controller({Dt, Angle, Speed}, {Adv_V_Goal, Adv_V_Ref}, {Turn_V_Goal, Turn_V_Ref}) ->
    {Pid_Speed, Pid_Stability} = persistent_term:get(controllers),

    Adv_V_Ref_New = saturate_acceleration(Adv_V_Goal, Adv_V_Ref, Dt, ?ADV_ACCEL, ?ADV_V_MAX),
    Turn_V_Ref_New = saturate_acceleration(Turn_V_Goal, Turn_V_Ref, Dt, ?TURN_ACCEL, ?TURN_V_MAX),

    % Speed PI
    Pid_Speed ! {self(), {set_point, Adv_V_Ref_New}},
    Pid_Speed ! {self(), {input, Speed}},
    receive {_, {control, Target_angle}} -> ok end,
    Timestamp = erlang:system_time(millisecond),

    % Adjust the target angle based on the speed: the robot should lean in the direction it is moving forward or backward
    % But the acceleration of the top and bottom of a rectangular paralelepided robot is not the same, so we need to adjust the angle accordingly

    % Stability PD
    Pid_Stability ! {self(), {set_point, Target_angle}},
    Pid_Stability ! {self(), {input, Angle}},
    receive {_, {control, Acc}} -> ok end,
    Acc_Limited = lists:max([-600.0, lists:min([Acc, 600.0])]),

    log_buffer:add({stability_controller, Timestamp, speed_controller, [Adv_V_Ref_New, Speed, Target_angle]}),
    log_buffer:add({stability_controller, Timestamp, stability_controller, [Target_angle, Angle, Acc_Limited]}),

    {Acc_Limited, Adv_V_Ref_New, Turn_V_Ref_New}.

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
