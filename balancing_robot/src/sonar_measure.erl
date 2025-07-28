-module(sonar_measure).
-behavior(hera_measure).

-export([init/1, measure/1]).

init([Role]) ->
    Name = list_to_atom("SONAR_" ++ atom_to_list(Role)),
    io:format("[SONAR] Starting ~p~n", [Role]),
    Tnow = erlang:system_time(millisecond),
    {ok, #{seq => 1, role => Role, last_distance => none, last_time => Tnow}, #{
        name => Name,
        iter => infinity,
        timeout => 50  %% => on attend uniquement les authorize
    }}.

measure(State) ->
    Role = maps:get(role, State),
    receive
        {authorize, Sender} ->
            Tnow = erlang:system_time(millisecond),
            PrevT = maps:get(last_time, State),
            Dt = Tnow - PrevT,

            RawD = measure_distance(),
            PrevD = maps:get(last_distance, State),
            FilteredD =
                case PrevD of
                    none ->
                        RawD;  % First measurement, no previous distance to filter
                    _ ->
                        % Filtering logic
                        Valid = RawD >= 15.0 andalso RawD =< 648.0,
                        if Valid -> filter(RawD, PrevD, Dt);
                        true -> PrevD end
                end,

            Seq = maps:get(seq, State),
            NewState = State#{
                seq => Seq + 1,
                last_distance => FilteredD,
                last_time => Tnow
            },
            io:format("[SONAR] New measure taken: ~p", [FilteredD]),
            send_to_main(Role, Sender, FilteredD, Seq),
            send_to_server(Role, FilteredD, Seq),
            {ok, [FilteredD], sonar_measure, Role, NewState}
    end.

measure_distance() ->
    Dist_inch = pmod_maxsonar:get(),
    Dist_cm = Dist_inch * 2.54,
    round_to(Dist_cm, 2).

round_to(Value, Precision) ->
    Factor = math:pow(10, Precision),
    round(Value * Factor) / Factor.

filter(New, Prev, Dt) ->
    Alpha = compute_alpha(Dt), % pondération dynamique selon Dt
    round_to(low_pass_filter(New, Prev, Alpha), 2).

compute_alpha(Dt) when Dt < 200 -> 0.7;
compute_alpha(Dt) when Dt < 500 -> 0.5;
compute_alpha(_) -> 0.3.

low_pass_filter(RawD, LastD, Alpha) ->
    case LastD of
      none -> RawD;
      _ -> Alpha * LastD + (1-Alpha)*RawD
    end.

send_to_main(Role, Sender, D, Seq) ->
    case Role of
        robot_main ->
            Sender ! {sonar_data, Role, [D, Seq]};
        _ ->
            Msg = "sonar_data , " ++ atom_to_list(Role) ++ " , " ++ float_to_list(D) ++ " , " ++ integer_to_list(Seq),
            hera_com:send_unicast(Sender, Msg, "UTF8")
    end.

send_to_server(Role, D, Seq) ->
    Msg = "sonar_data , " ++ atom_to_list(Role) ++ " , " ++ float_to_list(D) ++ " , " ++ integer_to_list(Seq),
    hera_com:send_unicast(server, Msg, "UTF8").
