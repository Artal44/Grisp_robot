-module(log_buffer).
-export([init/1, add/1, flush_to_server/2]).

%% Create two ETS tables: one for logs, one for metadata (max, idx)
init(MaxSize) ->
    ets:new(logs,     [named_table, set, public]),
    ets:new(log_meta, [named_table, set, public]),
    ets:insert(log_meta, {max, MaxSize}),
    ets:insert(log_meta, {count, 0}),
    %% Start idx at -1 so first increment yields 0
    ets:insert(log_meta, {idx, -1}).
    

add(Entry) ->
    [{max, Max}] = ets:lookup(log_meta, max),
    NewIndex = ets:update_counter(log_meta, idx, {2, 1, Max, 0}),
    ets:insert(logs, {NewIndex, Entry}),
    _ = ets:update_counter(log_meta, count, {2, 1, Max, Max}), % saturating at Max
    ok.

dump() ->
    [{idx, Last}]   = ets:lookup(log_meta, idx),
    [{count, Cnt}]  = ets:lookup(log_meta, count),
    [{max, Max}]    = ets:lookup(log_meta, max),
    case Cnt of
        0 -> [];
        _ ->
            Start = ((Last - Cnt + 1) rem Max + Max) rem Max,
            Indices = [((Start + I) rem Max) || I <- lists:seq(0, Cnt - 1)],
            [ {I, E} || I <- Indices, [{I, E}] <- [ets:lookup(logs, I)] ]
    end.

to_string(Value) when is_binary(Value) ->
    binary_to_list(Value);
to_string(Value) when is_atom(Value) ->
    atom_to_list(Value);
to_string(Value) ->
    io_lib:format("~p", [Value]).

flush_to_server(ServerRole, SelfRole) ->
    L = dump(),
    lists:foreach(fun({Index, Entry}) ->
        LogStr = format_log_entry(Entry),
        Msg = "log : " ++ atom_to_list(SelfRole) ++ " , " ++ lists:flatten(LogStr),
        hera_com:send_unicast(ServerRole, Msg, "UTF8"),
        ets:delete(logs, Index)  %% <-- delete the entry after printing
    end, L).

format_log_entry({Level, Timestamp, Category, Message}) ->
    io_lib:format("[~s] ~p | ~s | ~s",
        [string:to_upper(atom_to_list(Level)), Timestamp, to_string(Category), to_string(Message)]);
format_log_entry({Level, Timestamp, Message}) ->
    io_lib:format("[~s] ~p | ~s",
        [string:to_upper(atom_to_list(Level)), Timestamp, to_string(Message)]);
format_log_entry({Level, Timestamp}) ->
    io_lib:format("[~s] ~p", [string:to_upper(atom_to_list(Level)), Timestamp]);
format_log_entry(Other) ->
    io_lib:format("~p", [Other]).

