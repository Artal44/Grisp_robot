-module(kalman_computations).

-export([init_kalman/0, old_init_kalman/0, update_with_measurement/4, old_kalman_angle/6, kalman_predict_only/3, kalman_angle/7]).

-define(RAD_TO_DEG, 180.0/math:pi()).
-define(DEG_TO_RAD, math:pi()/180.0).

-define(g, 9.81). % Gravity in m/s²
-define(M, 3.4). % Mass of the robot (kg)
-define(h, 0.26). % Height of the robot center of mass (m)

-define(width, 0.185). % Width of the robot (m)
-define(height, 0.95). % Height of the robot (m)
-define(I, ?M * (math:pow(?width, 2) + math:pow(?height, 2)) / 12). % I = M * (w² + h²) / 12 (rectangular parallelepiped)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% KALMAN INITIALIZATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

calibrate_initial_state() ->
    N = 500, % Increase the number of samples for better accuracy
    Measurements = [pmod_nav:read(acc, [out_x_xl, out_z_xl, out_y_g]) || _ <- lists:seq(1, N)],
    {Ax_Sum, Az_Sum, Gy_Sum} = lists:foldl(
        fun ([Ax, Az, Gy], {Ax_Acc, Az_Acc, Gy_Acc}) ->
            {Ax_Acc + Ax, Az_Acc + Az, Gy_Acc + Gy}
        end,
        {0.0, 0.0, 0.0},
        Measurements
    ),
    Ax_Avg = Ax_Sum / N,
    Az_Avg = Az_Sum / N,
    Gy_Avg = Gy_Sum / N,

    % Compute the initial angle and angular velocity
    Initial_Angle = math:atan(Az_Avg / (-Ax_Avg)),
    Initial_Angular_Velocity = (Gy_Avg - persistent_term:get(gy0)) * ?DEG_TO_RAD, % Subtract gyroscope bias
    log_buffer:add({main_loop, erlang:system_time(millisecond), kalman_calibration, [Initial_Angle* ?RAD_TO_DEG, Initial_Angular_Velocity]}),
    {Initial_Angle, Initial_Angular_Velocity}.

init_kalman() ->
    % Adjusted Kalman constants
    R = mat:matrix([[3.0, 0.0], [0, 2.0e-6]]),
    Q = mat:matrix([[8.0e-5, 0.0], [0.0, 14.0]]),

    % Model constants
    G = ?g,
    Hh = ?h + (?I / (?M * ?h)),

    Jh = fun (_) -> mat:matrix([[1, 0], [0, 1]]) end,
    persistent_term:put(kalman_constant, {R, Q, Jh, G, Hh}),

    % Initial State and Covariance matrices
    {Initial_Angle, Initial_Angular_Velocity} = calibrate_initial_state(),
    X0 = mat:matrix([[Initial_Angle], [Initial_Angular_Velocity]]),
    P0 = mat:matrix([[0.01, 0], [0, 0.01]]), % Slightly increased initial covariance
    {X0, P0}.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% KALMAN COMPUTATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

update_with_measurement(Gy, Ax, Az, [Xk, Pk]) ->
    {R, _Q, Jh, _G, _Hh} = persistent_term:get(kalman_constant),
    H = fun (X) -> [Th, W] = mat:to_array(X), mat:matrix([[Th], [W]]) end,
    Z = mat:matrix([[math:atan(Az / (-Ax))], [(Gy - persistent_term:get(gy0)) * ?DEG_TO_RAD]]),
    {X1, P1} = kalman:ekf_correct({Xk, Pk}, H, Jh, R, Z),
    % kalman:ekf_control({X0, P0}, {F, Jf}, {H, Jh}, Q, R, Z, U)
    [Th_Kalman, _] = mat:to_array(X1),
    Angle = Th_Kalman * ?RAD_TO_DEG,
    [Angle, {X1, P1}].

kalman_predict_only(Dt, [Xk, Pk], Acc) ->
    {_R, Q, _Jh, G, Hh} = persistent_term:get(kalman_constant),
    F = fun (X, U) ->
        [Th, W] = mat:to_array(X),
        Th1 = Th + W * Dt,
        W1 = W + ((G / Hh) * math:sin(Th) - (U / Hh) * math:cos(Th)) * Dt,
        mat:matrix([[Th1], [W1]])
    end,
    Jf = fun (X) ->
        [Th, _] = mat:to_array(X),
        DW_dTh = ((G / Hh) * math:cos(Th) + (Acc / Hh) * math:sin(Th)) * Dt,
        mat:matrix([[1, Dt], [DW_dTh, 1]])
    end,
    kalman:ekf_predict({Xk, Pk}, F, Jf, Q, Acc). % {X1, P1}.

kalman_angle(Dt, Ax, Az, Gy, Acc, X0, P0) ->
    {R, Q, Jh, G, Hh} = persistent_term:get(kalman_constant),
    
    % Nonlinear state model (digital twin)
    F = fun (X, U) ->
        [Th, W] = mat:to_array(X),
        Th1 = Th + W * Dt,
        W1 = W + ((G / Hh) * math:sin(Th) - (U / Hh) * math:cos(Th)) * Dt,
        mat:matrix([[Th1], [W1]])
    end,

    % Jacobian of F
    Jf = fun (X) ->
        [Th, _W] = mat:to_array(X),
        DW_dTh = ((G / Hh) * math:cos(Th) + (Acc / Hh) * math:sin(Th)) * Dt,
        mat:matrix([[1, Dt],
                    [DW_dTh, 1]])
    end,

    % Observation function
    H = fun (X) ->
        [Th, W] = mat:to_array(X),
        mat:matrix([[Th], [W]])
    end,

    % Measurement vector: angle from accelerometer, angular velocity from gyro
    Z = mat:matrix([[math:atan(Az / (-Ax))], [(Gy - persistent_term:get(gy0)) * ?DEG_TO_RAD]]),
    {X1, P1} = kalman:ekf_control({X0, P0}, {F, Jf}, {H, Jh}, Q, R, Z, Acc),

    [Th_Kalman, _W_Kalman] = mat:to_array(X1),
    Angle = Th_Kalman * ?RAD_TO_DEG,
    [Angle, {X1, P1}].

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% OLD KALMAN COMPUTATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

old_init_kalman() ->
    % Initiating kalman constants
    R = mat:matrix([[3.0, 0.0], [0, 3.0e-6]]),
    Q = mat:matrix([[3.0e-5, 0.0], [0.0, 10.0]]),
    Jh = fun (_) -> mat:matrix([  	[1, 0],
								    [0, 1] ])
		 end,
    persistent_term:put(old_kalman_constant, {R, Q, Jh}),

    % Initial State and Covariance matrices
    X0 = mat:matrix([[0], [0]]),
    P0 = mat:matrix([[0.1, 0], [0, 0.1]]),
    {X0, P0}.

old_kalman_angle(Dt, Ax, Az, Gy, X0, P0) ->
    Gy0 = persistent_term:get(gy0),
    {R, Q, Jh} = persistent_term:get(old_kalman_constant),
    
    F = fun (X) -> [Th, W] = mat:to_array(X),
				mat:matrix([ 	[Th+Dt*W],
								[W      ] ])
		end,
    Jf = fun (_) -> mat:matrix([  	[1, Dt],
								    [0, 1 ] ])
		 end,
    H = fun (X) -> [Th, W] = mat:to_array(X),
				mat:matrix([ 	[Th],
								[W ] ])
		end,
    
    Z = mat:matrix([[math:atan(Az / (-Ax))], [(Gy-Gy0)*?DEG_TO_RAD]]),
    {X1, P1} = kalman:ekf({X0, P0}, {F, Jf}, {H, Jh}, Q, R, Z),

    [Th_Kalman, _W_Kalman] = mat:to_array(X1),
    Angle = Th_Kalman * ?RAD_TO_DEG,
    [Angle, {X1, P1}].
