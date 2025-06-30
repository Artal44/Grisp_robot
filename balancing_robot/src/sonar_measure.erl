-module(sonar_measure).

-behavior(hera_measure).

-export([init/1, measure/1]).

init([Pid, Role]) ->
    Name = list_to_atom("SONAR_" ++ atom_to_list(Role)),
    io:format("[SONAR] Starting ~p~n", [Role]),
    {ok, #{seq => 1, target => Pid, role => Role}, #{
        name => Name,
        iter => infinity,
        timeout => 100
    }}.

measure(State) ->
    receive
        {hera_com, "ping"} ->
            D = measure_distance(),
            Seq = maps:get(seq, State),
            NewState = State#{seq => Seq + 1},
            case maps:get(role, State) of
                robot_front_left ->
                    hera_com:send_unicast(robot_main, integer_to_list(D), "UTF8");
                robot_front_rigth ->
                    hera_com:send_unicast(robot_main, integer_to_list(D), "UTF8");
                robot_main ->
                    Pid = maps:get(target, State),
                    Pid ! {sonar_data, robot_main, [D]}
                end,
            {ok, [D], sonar_measure, maps:get(role, State), NewState}
    after 0 -> 
        {ok, [-1], sonar_measure, maps:get(role, State), State}
    end.

measure_distance() ->
    Dist_inch = pmod_maxsonar:get(),
    Dist_cm = Dist_inch * 2.54,
    D = round_to(Dist_cm, 4),
    D.
   
round_to(Value, Precision) ->
    Factor = math:pow(10, Precision),
    round(Value * Factor) / Factor.
