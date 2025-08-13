-module(sonar_measure).
-behaviour(hera_measure).

-export([init/1, measure/1]).

-define(ALPHA, 0.15).          
-define(MIN_CM, 15.0).
-define(MAX_CM, 648.0).

init([Role]) ->
    Name = list_to_atom("SONAR_" ++ atom_to_list(Role)),
    io:format("[SONAR] Starting ~p~n", [Role]),
    State = #{
        seq => 1,
        role => Role, 
        last_distance => none,
        hampel_buffer => []
    },                 
    {ok, State, #{name => Name, iter => infinity, timeout => 50}} .

measure(State) ->
    Role = maps:get(role, State),
    receive
        {authorize, Sender} ->
            RawD0 = measure_distance(),                       

            %% Hard gate
            Valid = (RawD0 >= ?MIN_CM) andalso (RawD0 =< ?MAX_CM),
            PrevD = maps:get(last_distance, State),
            RawD  = case Valid of true -> RawD0; false -> (PrevD =/= none andalso PrevD) orelse RawD0 end,

            % Lowa-pass filter + hampel + smoothnig
            LPF_filtered = low_pass_filter(RawD, PrevD, ?ALPHA),

            Final = round_to(LPF_filtered, 2),
            Seq = maps:get(seq, State),
            send_to_main(Role, Sender, Final, Seq),
            send_to_server(Role, Final, Seq),

            NewState = State#{
                    seq => Seq + 1,
                    last_distance => Final,
                    hampel_buffer => LPF_filtered
            },
            {ok, [Final], NewState}
        after 0 ->
            {ok, [-1], State}
    end.

%% --- helpers ---

measure_distance() ->
    Dist_inch = pmod_maxsonar:get(),
    Dist_inch * 2.54.

round_to(Value, Precision) ->
    Factor = math:pow(10, Precision),
    round(Value * Factor) / Factor.

low_pass_filter(RawD, LastD, Alpha) ->
    case LastD of
      none -> RawD;
      _    -> Alpha * LastD + (1-Alpha) * RawD
    end.

send_to_server(Role, D, Seq) ->
    Msg = "sonar_data , " ++ atom_to_list(Role) ++ " , " ++ float_to_list(D) ++ " , " ++ integer_to_list(Seq),
    hera_com:send_unicast(server, Msg, "UTF8").

send_to_main(Role, Sender, D, Seq) ->
    case Role of
        robot_main ->
            Sender ! {sonar_data, Role, [D, Seq]};
        _ ->
            Msg = "sonar_data , " ++ atom_to_list(Role) ++ " , " ++ float_to_list(D) ++ " , " ++ integer_to_list(Seq),
            hera_com:send_unicast(Sender, Msg, "UTF8")
    end.
