-module(balancing_robot).

-behavior(application).

-export([start/2, stop/1, dump_logs/0]).

% balancing_robot:dump_logs(). 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% GRiSP STARTUP %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

start(_Type, _Args) ->
    {ok, Supervisor} = balancing_robot_sup:start_link(),
    [grisp_led:flash(L, yellow, 500) || L <- [1, 2]],

    % Log buffer initialization
    log_buffer:init(5000),
    log_buffer:add({balancing_robot, erlang:system_time(millisecond), startup}),

    _ = grisp:add_device(spi2, pmod_nav),
    pmod_nav:config(acc, #{odr_g => {hz,238}}),

    _ = grisp:add_device(uart, pmod_maxsonar),
    numerl:init(),
    timer:sleep(2000),

    {ok, Id} = get_grisp_id(),
    case Id of
        0 ->
            log_buffer:add({balancing_robot, erlang:system_time(millisecond), main_robot}),
            spawn(main_loop, robot_init, []);
        1 ->
            log_buffer:add({balancing_robot, erlang:system_time(millisecond), left_sonar});
        2 ->
            log_buffer:add({balancing_robot, erlang:system_time(millisecond), right_sonar});
        _ ->
            log_buffer:add({balancing_robot, erlang:system_time(millisecond), unknown_id})
    end,

    {ok, Supervisor}.


stop(_State) -> 
    dump_logs(),
    ok.

dump_logs() ->
    log_buffer:dump_to_console().

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