-module(balancing_robot).

-behavior(application).

-export([start/2, stop/1]).

    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% GRiSP STARTUP %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

start(_Type, _Args) ->
    {ok, Supervisor} = balancing_robot_sup:start_link(),
    [grisp_led:flash(L, yellow, 500) || L <- [1, 2]],
	_ = grisp:add_device(spi2, pmod_nav),
    pmod_nav:config(acc, #{odr_g => {hz,238}}),

    _ = grisp:add_device(uart, pmod_maxsonar),
    numerl:init(),
    timer:sleep(2000),
    {ok, Id} = get_grisp_id(),
    if 
        Id == 0->
            io:format("[ROBOT] GRiSP ID : ~p~n", [Id]),
            spawn(main_loop, robot_init, []);
        Id == 1;
        Id == 2 ->
            io:format("[SONAR] GRiSP ID : ~p~n", [Id])
            % spawn(sonar_detection, sonar_init, [Id])
    end,
    
    hera_subscribe:subscribe(self()),
    config(),
    loop(),
    {ok, Supervisor}.

stop(_State) -> ok.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% NETWORK CONFIGURATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


config() ->
    persistent_term:put(name, list_to_atom("robot")),
    await_connection(),
    io:format("[ROBOT] Waiting for start signal ...~n~n").

await_connection() ->
    % Waiting for HERA to notify succesful connection
    io:format("[ROBOT] WiFi setup starting...~n"),
    receive        
        {hera_notify, "connected"} -> % Received when hera_com managed to connect to the network
            io:format("[ROBOT] WiFi setup done~n~n"),
            grisp_led:flash(2, white, 1000),
            discover_server()
    after 18000 ->
        io:format("[ROBOT] WiFi setup failed:~n~n"),
        grisp_led:flash(2, red, 750),
        await_connection()
    end.

discover_server() ->
    % Waits forever until the server sends a Ping
    io:format("[ROBOT] Waiting for ping from server~n"),
    receive
        {hera_notify, ["ping", Name, SIp, Port]} -> % Received upon server ping reception
            {ok, Ip} = inet:parse_address(SIp),
            IntPort = list_to_integer(Port),
            hera_com:add_device(list_to_atom(Name), Ip, IntPort),
            ack_loop()
    after 9000 ->
        io:format("[ROBOT] no ping from server~n"),
        discover_server()
    end.

ack_loop() ->
    % Tries to pair with the server by a Hello -> Ack
    % @param Id : Sensor's Id set by the jumpers (Integer)
    send_udp_message(server, "Hello from robot", "UTF8"),
    receive
        {hera_notify, ["Ack", _]} -> % Ensures the discovery of the sensor by the server
            io:format("[ROBOT] Received ACK from server~n"),
            [grisp_led:flash(L, green, 1000) || L <- [1, 2]],
            ok
    after 5000 ->
        ack_loop()
    end.

send_udp_message(Name, Message, Type) ->
    % Sends message
    % @param Name : name of the device to send to (atom)
    % @param Message : message to be sent (String/Tuple)
    % @param Type : type of message, can be UTF8 or Binary (String)
    hera_com:send_unicast(Name, Message, Type).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% LOOP %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

loop() ->
    receive 
        {hera_notify, ["Add_Device", Name, SIp, Port]} ->  % Received at config time to register all used sensors 
            add_device(Name, SIp, Port);          
        {hera_notify, ["Exit"]} ->
            reset_state();
        {hera_notify, ["ping", _, _, _]} -> % Ignore the pings after server discovery
            loop();
        {hera_notify, Msg} -> % Unhandled Message
            io:format("[SENSOR] Received unhandled message : ~p~n", [Msg]),
            loop();
        Msg -> % Message not from hera_notify
            io:format("[SENSOR] receive strange message : ~p~n",[Msg]),
            loop()
    end.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% LOOP FUNCTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

add_device(Name, SIp, SPort) ->
    % Adds a device to the list of known devices
    % @param Name : name of the device to register (String)
    % @param SIp : IP adress (String)
    % @param SPort : Port (String)
    case list_to_atom(Name) of 
        robot -> % Don't register self
            ok;
        OName -> 
            io:format("[SENSOR] Discovered new device : ~p~n", [Name]),
            {ok, Ip} = inet:parse_address(SIp),
            Port = list_to_integer(SPort),
            hera_com:add_device(OName, Ip, Port)
    end,            
    loop().

reset_state() ->
    % Kills all hera_measures modules, resets all data and jump back to server discovery
    % @param Id : Sensor's Id set by the jumpers (Integer)
    %exit_measure_module(sonar_sensor),
    exit_measure_module(kalman_measure),

    timer:sleep(500),
    reset_data(),

    grisp_led:flash(2, white, 1000),      
    grisp_led:flash(1, green, 1000),      

    discover_server(),            
    io:format("[SENSOR] Waiting for start signal ...~n~n"),
    loop().

reset_data() ->
    % Delete all config dependent and hera_measures data
    hera_data:reset(),
    io:format("[SENSOR] Data resetted~n~n").

exit_measure_module(Name) ->
    % Kills a module stored in persistent term
    % @param Name : the name of the module (atom)
    Pid = persistent_term:get(Name, none),    
    case Pid of
        none ->
            ok;
        _ -> 
            exit(Pid, shutdown)
    end.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% GRISP ID %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

get_grisp_id() ->
    % Computes the Id of the GRiSP board using the jumpers
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