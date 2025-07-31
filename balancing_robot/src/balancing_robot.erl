-module(balancing_robot).

-behavior(application).

-export([start/2, stop/1, dump_logs/0]).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% STARTUP
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

start(_Type, _Args) ->
    {ok, Supervisor} = balancing_robot_sup:start_link(),
    [grisp_led:flash(L, yellow, 500) || L <- [1, 2]],

    %% Initialize log buffer
    log_buffer:init(10000),
    log_buffer:add({balancing_robot, erlang:system_time(millisecond), startup}),

    numerl:init(),
    hera_subscribe:subscribe(self()),
    persistent_term:put(server_on, false),

    %% Get GRiSP ID and start correct processes
    {ok, Id} = get_grisp_id(),
    init_grisp(Id),

    %% Setup WiFi and server discovery
    config(),

    %% Start alive loop in its own process
    spawn(fun alive_loop/0),

    %% Keep process alive to handle Hera messages
    hera_notify_loop(),

    {ok, Supervisor}.

stop(_State) ->
    persistent_term:get(name),
    case persistent_term:get(name) of
        robot_main ->
            ets:delete(adv_goal_tab);
        _ ->
            ok
    end,
    dump_logs(),
    ok.

dump_logs() ->
    log_buffer:dump_to_console().

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% INITIALIZATION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

init_grisp(0) ->
    io:format("[BALANCING_ROBOT] GRiSP ID: 0, Spawning robot_main~n", []),
    persistent_term:put(name, robot_main),

    %% Initialize ETS table for adv_goal
    ets:new(adv_goal_tab, [set, public, named_table]),
    ets:insert(adv_goal_tab, {adv_goal, 0.0}),

    add_GRISP_device(spi2, pmod_nav),
    add_GRISP_device(uart, pmod_maxsonar),
    pmod_nav:config(acc, #{odr_g => {hz,238}}),
    timer:sleep(2000),

    log_buffer:add({balancing_robot, erlang:system_time(millisecond), robot_main}),
    Pid_Main = spawn(main_loop, robot_init, []),
    persistent_term:put(pid_main, Pid_Main),
    
    %% Start sonar scheduler
    start_sonar_scheduler(), 

    spawning_sonar(0, robot_main),
    hera:start_measure(nav_measure, [Pid_Main, robot_main]);

init_grisp(1) ->
    persistent_term:put(name, robot_front_left),
    add_GRISP_device(uart, pmod_maxsonar),
    timer:sleep(2000),
    spawning_sonar(1, robot_front_left);

init_grisp(2) ->
    persistent_term:put(name, robot_front_right),
    add_GRISP_device(uart, pmod_maxsonar),
    timer:sleep(2000),
    spawning_sonar(2, robot_front_right);

init_grisp(_) ->
    io:format("[BALANCING_ROBOT][ERROR] Unknown GRiSP ID~n", []),
    log_buffer:add({balancing_robot, erlang:system_time(millisecond), unknown_id}).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% DEVICES
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

add_GRISP_device(Port, Name) ->
    case catch grisp:add_device(Port, Name) of
        {device, _, _, _, _} = DeviceInfo ->
            [grisp_led:flash(L, yellow, 250) || L <- [1, 2]],
            io:format("[~p] Device ~p added (info: ~p)~n",
                      [persistent_term:get(name), Name, DeviceInfo]);
        Other ->
            [grisp_led:flash(L, red, 250) || L <- [1, 2]],
            io:format("[~p] Failed to add device ~p: ~p~n",
                      [persistent_term:get(name), Name, Other]),
            timer:sleep(2000),
            add_GRISP_device(Port, Name)
    end.

get_grisp_id() ->
    JMPs = [jumper_1, jumper_2, jumper_3, jumper_4, jumper_5],
    Bits = [grisp_gpio:get(grisp_gpio:open(J, #{mode => input})) || J <- JMPs],
    {ok, lists:foldl(fun(B, Acc) -> (Acc bsl 1) + B end, 0, lists:reverse(Bits))}.

spawning_sonar(Id, Role) ->
    log_buffer:add({balancing_robot, erlang:system_time(millisecond), Role}),
    {ok, Pid_Sonar} = hera:start_measure(sonar_measure, [Role]),
    persistent_term:put(pid_sonar, Pid_Sonar),
    io:format("[BALANCING_ROBOT] GRiSP ~p spawned ~p (PID: ~p)~n",
              [Id, Role, Pid_Sonar]).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% NETWORK / SERVER
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

config() -> await_connection().

await_connection() ->
    io:format("[~p] WiFi setup starting...~n", [persistent_term:get(name)]),
    receive
        {hera_notify, "connected"} ->
            io:format("[~p] WiFi connected~n", [persistent_term:get(name)]),
            [grisp_led:flash(L, white, 500) || L <- [1, 2]],
            discover_server()

    after 18000 ->
        io:format("[~p] WiFi setup failed. Retrying...~n", [persistent_term:get(name)]),
        [grisp_led:flash(L, magenta, 500) || L <- [1, 2]],
        await_connection()
    end.

discover_server() ->
    receive
        {hera_notify, ["ping", Name, SIp, Port]} ->
            {ok, Ip} = inet:parse_address(SIp),
            [grisp_led:flash(L, green, 1000) || L <- [1, 2]],
            hera_com:add_device(list_to_atom(Name), Ip, list_to_integer(Port)),
            ack_loop()
    after 9000 ->
        io:format("[~p] No ping from server. Retrying...~n", [persistent_term:get(name)]),
        [grisp_led:flash(L, red, 1000) || L <- [1, 2]],
        discover_server()
    end.

ack_loop() ->
    Payload = "Hello from " ++ atom_to_list(persistent_term:get(name)),
    hera_com:send_unicast(server, Payload, "UTF8"),
    receive
        {hera_notify, ["Ack", _]} ->
            persistent_term:put(server_on, true),
            [grisp_led:color(L, aqua) || L <- [1, 2]],
            io:format("[~p] Received ACK from server~n", [persistent_term:get(name)])
    after 5000 ->
        ack_loop()
    end.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% LOOPS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

alive_loop() ->
    Msg = "alive : " ++ atom_to_list(persistent_term:get(name)),
    hera_com:send_unicast(server, Msg, "UTF8"),
    timer:sleep(20000),
    alive_loop().

hera_notify_loop() ->
    receive
        {hera_notify, Msg} ->
            io:format("[~p] Received hera_notify: ~p~n", [persistent_term:get(name), Msg]),
            handle_hera_notify(Msg),
            hera_notify_loop();
        Other ->
            io:format("[~p] Unexpected message: ~p~n",
                      [persistent_term:get(name), Other]),
            hera_notify_loop()
    end.

handle_hera_notify(["ping", _, _, _]) -> ok;
handle_hera_notify(["Add_Device", Name, SIp, Port]) ->
    add_device(Name, SIp, Port);
handle_hera_notify(["authorize"]) ->
    persistent_term:get(pid_sonar) ! {authorize, robot_main};
handle_hera_notify(["sonar_data", Sonar_Name, D, Seq]) ->
    case persistent_term:get(pid_main, undefined) of
        undefined ->
            io:format("[~p] sonar_data ignored (no pid_main)~n",
                      [persistent_term:get(name)]);
        Pid_Main ->
            Pid_Main ! {sonar_data, list_to_atom(Sonar_Name),
                        [list_to_float(D), list_to_integer(Seq)]}
    end;
handle_hera_notify(Other) ->
    io:format("[~p] Unhandled hera_notify: ~p~n",
              [persistent_term:get(name), Other]).

add_device(Name, SIp, SPort) ->
    Self = persistent_term:get(name),
    case list_to_atom(Name) of
        Self -> ok;
        OName ->
            {ok, Ip} = inet:parse_address(SIp),
            hera_com:add_device(OName, Ip, list_to_integer(SPort)),
            io:format("[BALANCING_ROBOT] Added device ~p (IP: ~p, Port: ~p)~n",
                      [OName, Ip, SPort])
    end.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% SONAR SCHEDULER
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

start_sonar_scheduler() ->
    spawn(fun sonar_scheduler/0).

sonar_scheduler() ->
    receive
        after 50 -> % run every 50 ms
            [{adv_goal, Adv_V_Goal}] = ets:lookup(adv_goal_tab, adv_goal),
            handle_sonar_authorization(Adv_V_Goal),
            sonar_scheduler()
    end.

handle_sonar_authorization(Adv_V_Goal) ->
    case Adv_V_Goal of
        V when V > 0 ->
            % Recule : only back sonar
            persistent_term:get(pid_sonar) ! {authorize, persistent_term:get(pid_main)};
        V when V < 0 ->
            % Avance : alternate front_left / front_right
            Sonar_Role = persistent_term:get(current_sonar, robot_front_left),
            Next_Role = case Sonar_Role of
                robot_front_left  -> robot_front_right;
                robot_front_right -> robot_front_left
            end,
            persistent_term:put(current_sonar, Next_Role),
            spawn(fun() ->
                hera_com:send_unicast(Next_Role, "authorize", "UTF8")
            end);
        _ -> ok
    end.
