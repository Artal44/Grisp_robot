-module(obstacle_avoidance).
-export([compute_target_speed/3]).

%%% ------------------------------------------------
%%% Obstacle detection
%%% ------------------------------------------------

%% Constants
-define(D_SAFE, 50.0).    % Safe distance in cm
-define(D_MIN, 15.0).     % Minimum distance in cm
-define(EXPONENTIAL_K, 0.1). % Decay rate for exponential interpolation

%% @doc Computes the target speed based on the distance to an obstacle and the interpolation type.
%% @param Adv_V_Ref - The reference advance velocity.
%% @param Distance - The distance to the closest object in cm.
%% @param Type - The type of interpolation to use (linear, exponential).
%% @return - The computed target speed.
compute_target_speed(Adv_V_Ref, Distance, Type) ->
    case classify_distance(Distance) of
        safe ->
            Adv_V_Ref;
        stop ->
            0.0;
        slow_down ->
            interpolate_speed(Type, Adv_V_Ref, Distance)
    end.

%% @doc Classifies distance into zones.
%% @param Distance - The distance to the closest object in cm.
%% @return - The classification of the distance (safe, stop, slow_down).
classify_distance(Distance) when Distance >= ?D_SAFE ->
    safe;
classify_distance(Distance) when Distance =< ?D_MIN ->
    stop;
classify_distance(_) ->
    slow_down.

%% @doc Interpolates the target speed based on the chosen method.
%% @param Type - The type of interpolation to use (linear, exponential).
%% @param Adv_V_Ref - The reference advance velocity.
%% @param Distance - The distance to the closest object in cm.
%% @return - The interpolated speed.
interpolate_speed(linear, Adv_V_Ref, Distance) ->
    Factor = (Distance - ?D_MIN) / (?D_SAFE - ?D_MIN),
    Adv_V_Ref * Factor;

interpolate_speed(exponential, Adv_V_Ref, Distance) ->
    Normalized = (Distance - ?D_MIN) / (?D_SAFE - ?D_MIN),
    ExpFactor = math:exp(?EXPONENTIAL_K * (Normalized - 1)),
    % Clamp between 0 and Adv_V_Ref
    max(0, min(Adv_V_Ref * ExpFactor, Adv_V_Ref));

interpolate_speed(_, _Adv_V_Ref, _Distance) ->
    erlang:error({invalid_interpolation_type}).

