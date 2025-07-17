-module(log_buffer).
-export([init/1, add/1, dump_to_console/0, flush_to_server/2]).

init(MaxSize) ->
    ets:new(logs, [named_table, set, public]),
    persistent_term:put(log_max, MaxSize),
    persistent_term:put(log_index, 0).

add(Entry) ->
    Max = persistent_term:get(log_max),
    Index = persistent_term:get(log_index),
    NewIndex = (Index + 1) rem Max,
    ets:insert(logs, {NewIndex, Entry}),
    persistent_term:put(log_index, NewIndex).

dump() ->
    L = ets:tab2list(logs),
    lists:sort(
        fun({_, Entry1}, {_, Entry2}) ->
            TS1 = get_timestamp(Entry1),
            TS2 = get_timestamp(Entry2),
            TS1 < TS2
        end, L).

get_timestamp({_, TS, _, _}) -> TS;
get_timestamp({_, TS, _, _, _}) -> TS;
get_timestamp({_, TS}) -> TS;
get_timestamp(Entry) when is_tuple(Entry), tuple_size(Entry) >= 2 -> element(2, Entry).

%% Dumps all logs to console in order
dump_to_console() ->
    L = dump(),
    lists:foreach(fun({Index, Entry}) ->
        case Entry of
            {Level, Timestamp, Category, Message} ->
                CatStr = to_string(Category),
                MsgStr = to_string(Message),
                io:format("[~s] ~p | ~s | ~s~n",
                          [string:to_upper(atom_to_list(Level)), Timestamp, CatStr, MsgStr]);
            {Level, Timestamp, Message} ->
                MsgStr = to_string(Message),
                io:format("[~s] ~p | ~s~n",
                          [string:to_upper(atom_to_list(Level)), Timestamp, MsgStr]);
            {Level, Timestamp} ->
                io:format("[~s] ~p~n",
                          [string:to_upper(atom_to_list(Level)), Timestamp]);
            _ ->
                io:format("~p~n", [Entry])
        end,
        ets:delete(logs, Index)  %% <-- delete the entry after printing
    end, L).


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

