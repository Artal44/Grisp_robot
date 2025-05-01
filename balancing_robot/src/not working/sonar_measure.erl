-module(sonar_measure).

-behavior(hera_measure).

-export([init/1, measure/1]).

-define(INCH_TO_CM, 2.54). % Conversion factor from inch to cm

%============================================================================================================================================
%======================================================= HERA MEASURE BEHAVIOUR =============================================================
%============================================================================================================================================

init(_Args) ->
    io:format("[SONAR] Starting~n"),
    io:format("[SONAR] Starting measurements~n"),
    {ok, #{seq => 1}, #{
        name => sonar,
        iter => infinity,
        timeout => 1
    }}. 
    
measure(State) ->
    Distance_Sonar_inch = pmod_maxsonar:get(),
    Distance_Sonar_cm = Distance_Sonar_inch * ?INCH_TO_CM,
    D = round(Distance_Sonar_cm, 4),
    Seq = maps:get(seq, State),

    NewState = State#{seq => Seq + 1},
    %io:format("[SONAR] measured distance : ~p cm~n", [D]),
    
    hera_data:store(sonar, node(), Seq, [D]),
    {ok, [D], sonar, self(),  NewState}.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Math helpers
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% @doc Rounds a floating-point number to a specified number of decimal places.
%% @param Number - The number to round.
%% @param Precision - The number of decimal places to round to.
%% @return - The rounded number.
round(Number, Precision) ->
    Power = math:pow(10, Precision),
    round(Number * Power) / Power.