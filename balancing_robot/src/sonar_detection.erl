-module(sonar_detection).

-behavior(hera_measure).

-export([init/1, measure/1]).

init([Role]) ->
    Name = list_to_atom("SONAR_" ++ atom_to_list(Role)),
    io:format("[SONAR] Starting ~p~n", [Role]),
    {ok, #{seq => 1, name => Name, role => Role}, #{
        name => Name,
        iter => infinity,
        timeout => 100
    }}.

measure(State) ->
    receive 
        {hera_notify, ["authorized"]}->
            Dist_inch = pmod_maxsonar:get(),
            Dist_cm = Dist_inch * 2.54,
            D = round_to(Dist_cm, 4),
            Seq = maps:get(seq, State, 1),
            NewState = State#{seq => Seq + 1},
            io:format("[~p] Measured distance: ~p cm~n", [maps:get(name, State), D]),
            case maps:get(name, State) of
                robot_front_left ->
                    hera_com:send_unicast(robot_main, integer_to_list(D), "UTF8");
                robot_front_rigth ->
                    hera_com:send_unicast(robot_main, integer_to_list(D), "UTF8");
                robot_main ->
                    ok
            end,
            {ok, [D], maps:get(role, State), NewState};
        _ ->
            io:format("[~p] Received unexpected message in measure/1~n", [maps:get(name, State)]),
            {ok, [], maps:get(role, State), State}
    end.

round_to(Value, Precision) ->
    Factor = math:pow(10, Precision),
    round(Value * Factor) / Factor.
