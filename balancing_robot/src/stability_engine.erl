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

    % Stability PD
    Pid_Stability ! {self(), {set_point, Target_angle}},
    Pid_Stability ! {self(), {input, Angle}},
    receive {_, {control, Acc}} -> ok end,

    {Acc, Adv_V_Ref_New, Turn_V_Ref_New}.

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
