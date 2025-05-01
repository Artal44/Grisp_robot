% @doc sensor public API.
-module(sensor).

-behavior(application).

% Callbacks
-export([start/2, stop/1]).

%============================================================================================================================================
%========================================================= CONFIG SECTION ===================================================================
%============================================================================================================================================

% @private
start(_Type, _Args) -> 
    io:format("[SENSOR] start initialization sequence~n"),
    {ok, _} = sensor_sup:start_link(),
    hera_sub:subscribe(self()),
    [grisp_led:flash(L, yellow, 500) || L <- [1, 2]],
    grisp:add_device(uart, pmod_maxsonar),

    wifi_setup(),
    Id = persistent_term:get(id),
    io:format("[SENSOR] Waiting for start signal ...~n~n"),
    loop(Id),
    {ok, self()}.

% @private
stop(_State) -> ok.

wifi_setup() ->
    % Computing sensor id and storing it in persistent data
    {ok, Id} = get_grisp_id(),
    io:format("[SENSOR] sensor id :~p~n",[Id]),
    persistent_term:put(sensor_name, list_to_atom("sensor_" ++ integer_to_list(Id))),
    persistent_term:put(id, Id),
    await_connection(Id).

await_connection(Id) ->
    io:format("[SENSOR] WiFi setup starting...~n"),
    receive        
        {hera_notify, "connected"} -> % Received when hera_com managed to connect to the network
            io:format("[SENSOR] WiFi setup done~n~n"),
            [grisp_led:flash(L, green, 1000) || L <- [1, 2]],
            discover_server(Id)
    after 18000 ->
        io:format("[SENSOR] WiFi setup failed:~n~n"),
        [grisp_led:flash(L, red, 750) || L <- [1, 2]],
        await_connection(Id)
    end.

discover_server(Id) ->
    io:format("[SENSOR] Waiting for ping from server~n"),
    receive
        {hera_notify, ["ping", Name, SIp, Port]} -> % Received upon server ping reception
            {ok, Ip} = inet:parse_address(SIp),
            IntPort = list_to_integer(Port),
            hera_com:add_device(list_to_atom(Name), Ip, IntPort),
            ack_loop(Id)
    after 9000 ->
        io:format("[SENSOR] no ping from server~n"),
        discover_server(Id)
    end.

ack_loop(Id) ->
    send_udp_message(server, "Hello from " ++ integer_to_list(Id), "UTF8"),
    receive
        {hera_notify, ["Ack", _]} -> % Ensures the discovery of the sensor by the server
            io:format("[SENSOR] Received ACK from server~n"),
            ok
    after 5000 ->
        ack_loop(Id)
    end.

get_grisp_id() ->
    JMP1 = grisp_gpio:open(jumper_1, #{mode => input}),
    JMP2 = grisp_gpio:open(jumper_2, #{mode => input}),
    JMP3 = grisp_gpio:open(jumper_3, #{mode => input}),
    JMP4 = grisp_gpio:open(jumper_4, #{mode => input}),
    JMP5 = grisp_gpio:open(jumper_5, #{mode => input}),

    V1 = grisp_gpio:get(JMP1),
    V2 = grisp_gpio:get(JMP2),
    V3 = grisp_gpio:get(JMP3),
    V4 = grisp_gpio:get(JMP4),
    V5 = grisp_gpio:get(JMP5),

    SUM = (V1) + (V2 bsl 1) + (V3 bsl 2) + (V4 bsl 3) + (V5 bsl 4),
    {ok, SUM}.

reset_data() ->
    persistent_term:erase(osensor),
    persistent_term:erase(sonar_sensor),
    hera_data:reset(),
    io:format("[SENSOR] Data resetted~n~n").
    
%============================================================================================================================================
%============================================================== LOOP ========================================================================
%============================================================================================================================================

loop(Id) ->
    receive
        {hera_notify, ["Add_Device", Name, SIp, Port]} ->  % Received at config time to register all used devices           
            SelfName = persistent_term:get(sensor_name),
            case list_to_atom(Name) of 
                SelfName -> % Don't register self
                    ok;
                OName -> 
                    io:format("[SENSOR] Discovered new device : ~p~n", [Name]),
                    {ok, Ip} = inet:parse_address(SIp),
                    IntPort = list_to_integer(Port),
                    hera_com:add_device(OName, Ip, IntPort)
            end,            
            loop(Id);

        {hera_notify, ["Pos", Ids, Xs, Ys, RoomS]} -> % Received at config time To get all the sensors positions            
            X = list_to_float(Xs),
            Y = list_to_float(Ys),
            Room = list_to_integer(RoomS),
            SensorName = list_to_atom("sensor_" ++ Ids),
            hera_data:store(room, SensorName, 1, [Room]),
            hera_data:store(pos, SensorName, 1, [X, Y]),
            %io:format("[SENSOR] Sensor's ~p position : (~p,~p) in room n°~p~n",[ParsedId,X,Y, Room]),
            loop(Id);
        {hera_notify, ["Start", _]} -> % Received at the end of the configuration to launch the simulation
            spawn(target_angle, start_link, [Id]),
            timer:sleep(300),
            {ok, Pid} = hera:start_measure(sonar_sensor, []),
            persistent_term:put(sonar_sensor, Pid),
            [grisp_led:color(L, green) || L <- [1, 2]],
            loop(Id);
        {hera_notify, ["measure"]} -> % Received from an other sensor when it wants to measure
            Pid = persistent_term:get(sonar_sensor, none),
            case Pid of
                none ->
                    io:format("[ERROR] Sonar sensor has not spawned~n"),
                    loop(Id);
                _ ->
                    Pid ! {measure},
                    loop(Id)
            end;
        {hera_notify, ["Exit"]} ->
            SensorID = persistent_term:get(sonar_sensor),
            [grisp_led:flash(L, green, 1000) || L <- [1, 2]],
            exit(SensorID, shutdown),
            reset_data(),            
            discover_server(Id),
            io:format("[SENSOR] Waiting for start signal ...~n~n"),
            loop(Id);
        {hera_notify, ["ping", _, _, _]} -> % Ignore the pings after server discovery
            loop(Id);
        {hera_notify, Msg} ->
            io:format("[SENSOR] Received unhandled message : ~p~n", [Msg]),
            loop(Id);
        Msg ->
            io:format("[SENSOR] receive strange message : ~p~n",[Msg]),
            loop(Id)
    end.


%============================================================================================================================================
%======================================================== HELPER FUNCTIONS ==================================================================
%============================================================================================================================================

send_udp_message(Name, Message, Type) ->
    hera_com:send_unicast(Name, Message, Type).