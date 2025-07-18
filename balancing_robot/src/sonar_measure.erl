-module(sonar_measure).

%% -behavior(hera_measure).

-export([init/1, measure/1]).

init([Role]) ->
    Name = list_to_atom("SONAR_" ++ atom_to_list(Role)),
    io:format("[SONAR] Starting ~p~n", [Role]),

    {ok, #{seq => 1, role => Role}, #{
        name => Name,
        iter => infinity,
        timeout => 50
    }}.

measure(State) ->
    receive
        {authorize, Sender} ->
            D = measure_distance(),
            Seq = maps:get(seq, State),
            NewState = State#{seq => Seq + 1},

            Role = maps:get(role, State),
            case Role of
                robot_main ->
                    % Sender is the main_loop Pid
                    Msg = {sonar_data, maps:get(role, State), [D, Seq]},
                    Sender ! Msg;
                _ ->
                    % Sender is the main grisp name device (robot_main)
                    Msg = "sonar_data , " ++ atom_to_list(Role) ++ " , " ++ float_to_list(D) ++ " , " ++ integer_to_list(Seq),
                    hera_com:send_unicast(Sender, Msg, "UTF8")
            end,
            {ok, [D], sonar_measure, maps:get(role, State), NewState}
    end.

measure_distance() ->
    Dist_inch = pmod_maxsonar:get(),
    Dist_cm = Dist_inch * 2.54,
    round_to(Dist_cm, 4).
   
round_to(Value, Precision) ->
    Factor = math:pow(10, Precision),
    round(Value * Factor) / Factor.
