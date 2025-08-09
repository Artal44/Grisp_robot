-module(stability_engine).

-export([controller/5]).

-define(ADV_V_MAX, 16.0).
-define(ADV_ACCEL, 8.0).

-define(TURN_V_MAX, 40.0).
-define(TURN_ACCEL, 200.0).

-define(MIN_SONAR_DIST, 0.35).   
-define(MAX_SONAR_DIST, 0.55).   

-define(ANGLE_OFFSET, 0.6).

controller({Dt, Angle, Speed}, {Sonar_Data, Direction}, {Adv_V_Goal, Adv_V_Ref}, {Turn_V_Goal, Turn_V_Ref}, DoLog) ->
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% CONTROLLER %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    {Pid_Speed, Pid_Stability} = persistent_term:get(controllers),

    % Évitement automatique si bloqué
    Turn_V_Goal_Avoid =
        case Sonar_Data =< ?MIN_SONAR_DIST andalso (Adv_V_Goal > 0 orelse Adv_V_Goal < 0) of
            true -> ?TURN_V_MAX + 20;  % tourne à droite (ou -?TURN_V_MAX à gauche)
            false -> Turn_V_Goal
        end,

    % Applique un freinage si l’obstacle est détecté
    Adv_V_Goal_Safe =
        case Sonar_Data of 
            none ->
                Adv_V_Goal;  % Pas de donnée sonar, on garde la vitesse
            _ ->
                case Sonar_Data < ?MAX_SONAR_DIST of
                    true -> 
                        case Adv_V_Goal of
                            V when V < 0.0 andalso Direction =:= front ->
                                0.0;  % Stop en marche avant
                            V when V > 0.0 andalso Direction =:= back ->
                                0.0;  % Stop en marche arrière
                            _ -> 
                                Adv_V_Goal
                        end;
                    false ->
                        Adv_V_Goal
                end
        end,

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ACCELERATION SATURATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Adv_V_Ref_New = saturate_acceleration(Adv_V_Goal_Safe, Adv_V_Ref, Dt, ?ADV_ACCEL, ?ADV_V_MAX),
    Turn_V_Ref_New = saturate_acceleration(Turn_V_Goal_Avoid, Turn_V_Ref, Dt, ?TURN_ACCEL, ?TURN_V_MAX),

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ADVANCED SPEED CONTROLLER %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Pid_Speed ! {self(), {set_point, Adv_V_Ref_New}},
    Pid_Speed ! {self(), {input, Speed}},
    receive {_, {control, Target_Angle}} -> ok end,
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% STABILITY CONTROLLER %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    Corrected_Angle = Angle - ?ANGLE_OFFSET,
    Pid_Stability ! {self(), {set_point, Target_Angle}},
    Pid_Stability ! {self(), {input, Corrected_Angle}},
    receive {_, {control, Acc}} -> ok end,

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% LOGGING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    add_log({controller, erlang:system_time(millisecond), speed_controller, [Adv_V_Ref_New, Speed, Target_Angle]}, DoLog),
    add_log({controller, erlang:system_time(millisecond), stability_controller, [Target_Angle, Corrected_Angle, Acc]}, DoLog),
    
    {Acc, Adv_V_Ref_New, Turn_V_Ref_New}.

% Saturates the acceleration based on the goal and reference speed.
% If the goal is positive, it accelerates towards the goal, if negative, it decelerates.
% If the goal is zero, it checks the reference speed and applies a deceleration or acceleration based on the current speed.
saturate_acceleration(Goal, Ref, Dt, Accel, V_Max) ->
    case Goal of
        G when G > 0.0 ->
            hera_pid_controller:saturation(Ref + Accel * Dt, V_Max);
        G when G < 0.0 ->
            hera_pid_controller:saturation(Ref - Accel * Dt, V_Max);
        _ ->
            case Ref of
                R when R > 0.05 ->
                    hera_pid_controller:saturation(Ref - Accel * Dt, V_Max);
                R when R < -0.05 ->
                    hera_pid_controller:saturation(Ref + Accel * Dt, V_Max);
                _ ->
                    0.0
            end
    end.

add_log(Log, DoLog) ->
    case DoLog of
        true ->
            log_buffer:add(Log);
        false -> ok
    end.
