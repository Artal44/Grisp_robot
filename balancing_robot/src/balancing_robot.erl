-module(balancing_robot).

-behavior(application).

-export([start/2, stop/1]).

    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% GRiSP STARTUP %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

start(_Type, _Args) ->
    {ok, Supervisor} = balancing_robot_sup:start_link(),
    [grisp_led:flash(L, yellow, 500) || L <- [1, 2]],
    io:format("[BALANCING_ROBOT] Starting GRiSP application~n", []),
    numerl:init(),

    {ok, Id} = get_grisp_id(),
    if 
        Id == 0->
            io:format("[ROBOT_MAIN] GRiSP ID : ~p~n", [Id]),
            persistent_term:put(name, robot_main),
            add_GRISP_device(spi2, pmod_nav),
            pmod_nav:config(acc, #{odr_g => {hz,238}}),
            add_GRISP_device(uart, pmod_maxsonar),
            timer:sleep(2000),
            spawn(main_loop, robot_init, []);
        Id == 1 ->
            io:format("[ROBOT_FRONT_LEFT] GRiSP ID : ~p~n", [Id]),
            persistent_term:put(name, robot_front_left),
            add_GRISP_device(uart, pmod_maxsonar),
            timer:sleep(2000);
        Id == 2 ->
            io:format("[ROBOT_FRONT_RIGHT] GRiSP ID : ~p~n", [Id]),
            persistent_term:put(name, robot_front_right),
            add_GRISP_device(uart, pmod_maxsonar),
            timer:sleep(2000)
    end,
    
    hera_subscribe:subscribe(self()),
    config(),
    loop(),
    {ok, Supervisor}.

stop(_State) -> ok.

add_GRISP_device(Port, Name) ->
    case grisp:add_device(Port, Name) of
        {device, _, _, _, _} = DeviceInfo ->
            io:format("[~p] Device ~p added (info: ~p)~n", [persistent_term:get(name), Name, DeviceInfo]);
        {error, Reason} ->
            io:format("[~p] Device ~p not added: ~p~n", [persistent_term:get(name), Name, Reason]),
            timer:sleep(2000),
            add_GRISP_device(Port, Name);
        Other ->
            io:format("[~p] Unexpected return from grisp:add_device(~p, ~p): ~p~n", [persistent_term:get(name), Port, Name, Other]),
            timer:sleep(2000),
            add_GRISP_device(Port, Name)
    end.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% NETWORK CONFIGURATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

config() ->
    await_connection(),
    io:format("[~p] Waiting for start signal ...~n~n", [persistent_term:get(name)]).

await_connection() ->
    % Waiting for HERA to notify succesful connection
    io:format("[~p] WiFi setup starting...~n", [persistent_term:get(name)]),
    receive        
        {hera_notify, "connected"} -> % Received when hera_com managed to connect to the network
            io:format("[~p] WiFi setup done~n~n", [persistent_term:get(name)]),
            grisp_led:flash(2, white, 1000),
            discover_server()
    after 18000 ->
        io:format("[~p] WiFi setup failed:~n~n", [persistent_term:get(name)]),
        grisp_led:flash(2, red, 750),
        await_connection()
    end.

discover_server() ->
    % Waits forever until the server sends a Ping
    io:format("[~p] Waiting for ping from server~n", [persistent_term:get(name)]),
    receive
        {hera_notify, ["ping", Name, SIp, Port]} -> % Received upon server ping reception
            {ok, Ip} = inet:parse_address(SIp),
            IntPort = list_to_integer(Port),
            hera_com:add_device(list_to_atom(Name), Ip, IntPort),
            ack_loop()
    after 9000 ->
        io:format("[~p] no ping from server~n", [persistent_term:get(name)]),
        discover_server()
    end.

ack_loop() ->
    % Tries to pair with the server by a Hello -> Ack
    Payload = "Hello from " ++ atom_to_list(persistent_term:get(name)),
    send_udp_message(server, Payload, "UTF8"),
    receive
        {hera_notify, ["Ack", _]} -> % Ensures the discovery of the sensor by the server
            io:format("[~p] Received ACK from server~n", [persistent_term:get(name)]),
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
        {hera_notify, ["Start", _]} -> % Received at the end of the configuration to launch the simulation
            start_measures();
        {hera_notify, ["Exit"]} -> % Received when gracefully exited the controller
            io:format("~n[~p] Exit message received~n", [persistent_term:get(name)]),
            reset_state();
        {hera_notify, ["ping", _, _, _]} -> % Ignore the pings after server discovery
            loop();
        {hera_notify, Msg} -> % Unhandled Message
            io:format("[~p] Received unhandled message : ~p~n", [persistent_term:get(name), Msg]),
            loop();
        Msg -> % Message not from hera_notify
            io:format("[~p] receive strange message : ~p~n",[persistent_term:get(name), Msg]),
            loop()
    end.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% LOOP FUNCTIONS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

add_device(Name, SIp, SPort) ->
    % Adds a device to the list of known devices
    % @param Id : Sensor's Id set by the jumpers (Integer)
    % @param Name : name of the device to register (String)
    % @param SIp : IP adress (String)
    % @param SPort : Port (String)
    SelfName = persistent_term:get(name),
    case list_to_atom(Name) of 
        SelfName -> % Don't register self
            ok;
        OName ->             
            {ok, Ip} = inet:parse_address(SIp),
            Port = list_to_integer(SPort),
            hera_com:add_device(OName, Ip, Port)      
    end,            
    loop().

start_measures() ->
    % Launch all the hera_measure modules to gather data
    io:format("=================================================================================================~n"),
    io:format("~n~n[~p] Start received, starting the computing phase~n", [persistent_term:get(name)]),            
    {ok, Sonar_Pid} = hera:start_measure(sonar_detection, [persistent_term:get(name)]),
    persistent_term:put(sonar_detection, Sonar_Pid),
    [grisp_led:color(L, green) || L <- [1, 2]],
    loop(). 

reset_state() ->
    % Kills all hera_measures modules, resets all data and jump back to server discovery
    %exit_measure_module(sonar_sensor),
    exit_measure_module(sonar_detection),

    timer:sleep(500),
    reset_data(),

    grisp_led:flash(2, white, 1000),      
    grisp_led:flash(1, green, 1000),      

    discover_server(),            
    io:format("[~p] Waiting for start signal ...~n~n", [persistent_term:get(name)]),
    loop().

reset_data() ->
    % Delete all config dependent and hera_measures data
    persistent_term:erase(sonar_detection),
    hera_com:reset_devices(), 
    hera_data:reset(),
    io:format("[~p] Data resetted~n~n~n~n", [persistent_term:get(name)]),
    io:format("=================================================================================================~n").

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