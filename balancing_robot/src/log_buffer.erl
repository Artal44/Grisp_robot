-module(log_buffer).
-export([init/1, add/1, dump_to_console/0]).

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
    lists:sort(fun({I1,_}, {I2,_}) -> I1 < I2 end, L).

%% Dumps all logs to console in order
dump_to_console() ->
    L = dump(),
    lists:foreach(fun({_, {Level, Timestamp, Category, Message}}) ->
        CatStr = to_string(Category),
        MsgStr = to_string(Message),
        io:format("[~s] ~p | ~s | ~s~n",
            [string:to_upper(atom_to_list(Level)), Timestamp, CatStr, MsgStr])
    end, L).

to_string(Value) when is_binary(Value) ->
    binary_to_list(Value);
to_string(Value) when is_atom(Value) ->
    atom_to_list(Value);
to_string(Value) ->
    io_lib:format("~p", [Value]).


    
