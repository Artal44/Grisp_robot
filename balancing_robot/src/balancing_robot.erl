-module(balancing_robot).

-behavior(application).

-export([start/2, stop/1, dump_logs/0]).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% GRiSP STARTUP %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

start(_Type, _Args) ->
    {ok, Supervisor} = balancing_robot_sup:start_link(),

    % Log buffer initialization
    log_buffer:init(10000),
    log_buffer:add({balancing_robot, erlang:system_time(millisecond), startup}),

    numerl:init(),  

    hera_subscribe:subscribe(self()),
    persistent_term:put(server_on, false),

    {ok, Id} = get_grisp_id(),
    case Id of
        0 ->
            io:format("[BALANCING_ROBOT] GRiSP ID: ~p, Spawning robot_main~n", [Id]),
            persistent_term:put(name, robot_main),
            
            % PMODS initialization
            add_GRISP_device(spi2, pmod_nav),
            add_GRISP_device(uart, pmod_maxsonar),
            pmod_nav:config(acc, #{odr_g => {hz,238}}),
            timer:sleep(5000),

            % Main process spawning 
            log_buffer:add({balancing_robot, erlang:system_time(millisecond), robot_main}),
            Pid_Main = spawn(main_loop, robot_init, []),
            persistent_term:put(pid_main, Pid_Main),

            % Sonar and nav hera_measure spawning
            spawning_sonar(0, robot_main),
            hera:start_measure(nav_measure, [Pid_Main, robot_main]);
        1 ->
            [grisp_led:color(L, magenta) || L <- [1, 2]],
            persistent_term:put(name, robot_front_left),

            % PMODS initialization and sonar measure spawn
            add_GRISP_device(uart, pmod_maxsonar),
            timer:sleep(5000),
            spawning_sonar(1, robot_front_left);
        2 ->
            [grisp_led:color(L, magenta) || L <- [1, 2]],
            persistent_term:put(name, robot_front_right),

            % PMODS initialization and sonar measure spawn
            add_GRISP_device(uart, pmod_maxsonar),
            timer:sleep(5000),
            spawning_sonar(2, robot_front_right);
        _ ->
            io:format("[BALANCING_ROBOT][ERROR] Unknown GRiSP ID: ~p~n", [Id]),
            log_buffer:add({balancing_robot, erlang:system_time(millisecond), unknown_id})
    end,

    config(),
    loop(),
    {ok, Supervisor}.

stop(_State) -> 
    dump_logs(),
    ok.

dump_logs() ->
    log_buffer:dump_to_console().

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% GRISP ID %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

add_GRISP_device(Port, Name) ->
    case catch grisp:add_device(Port, Name) of
        {device, _, _, _, _} = DeviceInfo ->
            io:format("[~p] Device ~p added (info: ~p)~n", [persistent_term:get(name), Name, DeviceInfo]);
        {error, Reason} ->
            io:format("[~p] Device ~p not added: ~p~n", [persistent_term:get(name), Name, Reason]),
            timer:sleep(2000),
            add_GRISP_device(Port, Name);
        {'EXIT', _Reason} ->
            io:format("[~p] grisp:add_device(~p, ~p) crashed, restarting node...~n", [persistent_term:get(name), Port, Name]),
            init:restart();
        Other ->
            io:format("[~p] Unexpected return from grisp:add_device(~p, ~p): ~p~n", [persistent_term:get(name), Port, Name, Other]),
            timer:sleep(2000),
            add_GRISP_device(Port, Name)
    end.

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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PMODs %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
spawning_sonar(Id, Role) ->
    % Front left sonar initialization
    log_buffer:add({balancing_robot, erlang:system_time(millisecond), Role}),
    {ok, Pid_Sonar} = hera:start_measure(sonar_measure, [Role]),
    io:format("[BALANCING_ROBOT] GRiSP ID: ~p, Spawned ~p with PID: ~p~n", [Id, Role, Pid_Sonar]),
    persistent_term:put(pid_sonar, Pid_Sonar).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% NETWORK CONFIGURATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

config() ->
    await_connection().
await_connection() ->
    % Waiting for HERA to notify succesful connection
    io:format("[~p] WiFi setup starting...~n", [persistent_term:get(name)]),
    receive        
        {hera_notify, "connected"} -> % Received when hera_com managed to connect to the network
            io:format("[~p] WiFi setup done~n~n", [persistent_term:get(name)]),
            discover_server()
    after 18000 ->
        io:format("[~p] WiFi setup failed:~n~n", [persistent_term:get(name)]),
        await_connection()
    end.

discover_server() ->
    % Waits forever until the server sends a Ping
    io:format("[~p] Waiting for ping from server~n", [persistent_term:get(name)]),
    receive
        {hera_notify, ["ping", Name, SIp, Port]} -> % Received upon server ping reception
            io:format("[~p] Received ping from server: ~p, IP: ~p, Port: ~p~n", [persistent_term:get(name), Name, SIp, Port]),
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
    hera_com:send_unicast(server, Payload, "UTF8"),
    receive
        {hera_notify, ["Ack", _]} -> % Ensures the discovery of the sensor by the server
            io:format("[~p] Received ACK from server~n", [persistent_term:get(name)]),
            start_alive_loop(),
            persistent_term:put(server_on, true),
            ok       
    after 5000 ->
        ack_loop()
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
            hera_com:add_device(OName, Ip, Port),
            io:format("[BALANCING_ROBOT] Adding device: ~p, IP: ~p, Port: ~p~n", [OName, Ip, SPort])     
    end.

start_alive_loop() ->
    spawn(fun alive_loop/0).

alive_loop() ->
    Msg = "alive : " ++ atom_to_list(persistent_term:get(name)),
    hera_com:send_unicast(server, Msg, "UTF8"),
    timer:sleep(25000),
    alive_loop().

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% LOOP %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

loop() ->
    receive 
        {hera_notify, Msg} ->
            % io:format("[~p] Received hera_notify message: ~p~n", [persistent_term:get(name), Msg]),
            case Msg of
                ["ping", _, _, _] ->
                    ok;
                ["Add_Device", Name, SIp, Port] ->
                    add_device(Name, SIp, Port);
                ["authorize"] ->
                    % io:format("[~p] Received authorize, forwarding to sonar~n", [persistent_term:get(name)]),
                    persistent_term:get(pid_sonar) ! {authorize, robot_main};
                ["sonar_data", Sonar_Name, D, Seq] ->
                    % io:format("[~p] Received sonar data from ~p: ~p, Seq: ~p~n", [persistent_term:get(name), Sonar_Name, D, Seq]),
                    persistent_term:get(pid_main) ! {sonar_data, list_to_atom(Sonar_Name), [list_to_float(D), list_to_integer(Seq)]};
                _ ->
                    io:format("[~p] Unhandled content: ~p~n", [persistent_term:get(name), Msg])
            end,
            loop();
        Other ->
            io:format("[~p] Unexpected message: ~p~n", [persistent_term:get(name), Other]),
            loop()
    after 0 ->
        loop()
    end.
