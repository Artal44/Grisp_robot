-module(sonar_sensor).

-behavior(hera_measure).

-export([init/1, measure/1]).

init(_Args) ->
    io:format("~n[SONAR_SENSOR] Starting measurements~n"),
    {ok, #{seq => 1}, #{
        name => sonar_sensor,
        iter => infinity,
        timeout => 300
    }}.
    
measure(State) ->
    SensorName = persistent_term:get(sensor_name),
    {ok, N} = get_rand_num(),
    case persistent_term:get(osensor, none) of
        none -> % Is alone in a room
            io:format("[SONAR_SENOSR] Alone in room, measuring~n"),
            get_measure(State, SensorName);                      
        Osensor ->
            receive
                {measure} -> % Received when an other sensor wants to measure
                    io:format("[SONAR_SENSOR] possible collision, waiting for ~pms~n",[N]),
                    timer:sleep(N div 2 + 50),
                    get_measure(State, SensorName)            
            after N + 50 -> % Timeout, can measure
                io:format("[SONAR_SENOSR] Timeout~n"),                
                hera_com:send_unicast(Osensor, "measure", "UTF8"),
                get_measure(State, SensorName)
            end
        end.            
            

get_measure(State, SensorName) ->
    Seq = maps:get(seq, State, 1),
    Dist_inch = pmod_maxsonar:get(),
    Dist_cm = Dist_inch * 2.54,
    Measure = round(Dist_cm, 4),    
    %io:format("[SONAR_SENSOR] Sonar measure ~p : ~p~n", [Seq, R_Dist_cm]),
    hera_data:store(distance, SensorName, Seq, [Measure]),
    NewState = State#{seq => Seq + 1},
    {ok, [Measure], distance, SensorName, NewState}.
    

round(Number, Precision) ->
    Power = math:pow(10, Precision),
    round(Number * Power) / Power.

get_rand_num() ->
    Seed = {erlang:monotonic_time(), erlang:unique_integer([positive]), erlang:phash2(node())},
    rand:seed(exsplus, Seed),
    {ok, rand:uniform(150)}.   