-module(nav_measure).
-behavior(hera_measure).

-export([init/1, measure/1]).

init([Pid, Role]) ->
    Name = list_to_atom("NAV_" ++ atom_to_list(Role)),
    hera:logg("[NAV] Starting ~p~n", [Role]),
    {ok, #{seq => 1, target => Pid, role => Role}, #{
        name => Name,
        iter => infinity,
        timeout => 5 % ODR is 238 Hz, the sensor updates every ~4.2 ms.
      }}.

measure(State) ->
    try
        [Gy, Ax, Az] = pmod_nav:read(acc, [out_y_g, out_x_xl, out_z_xl], #{g_unit => dps}),
        Pid = maps:get(target, State),
        Pid ! {nav_data, [Gy, Ax, Az]},
        Seq = maps:get(seq, State),
        NewState = State#{seq => Seq + 1},
        {ok, [Gy, Ax, Az], nav_measure, maps:get(role, State), NewState}
    catch
        _:Error ->
            hera:logg("[NAV][ERROR] ~p~n", [Error]),
            {ok, [], maps:get(role, State), State}
    end.
