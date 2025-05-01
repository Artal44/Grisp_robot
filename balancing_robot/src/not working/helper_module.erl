-module(helper_module).

-export([complem_angle/1, select_angle/3, speed_ref/2, turn_ref/2, frequency_computation/4, wait/1, flicker_led/2, get_grisp_id/0]).
-define(FREQ_WINDOW, 100).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Macros
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Advance constant
-define(ADV_V_MAX, 30.0).

%Turning constant
-define(TURN_V_MAX, 80.0).

%Angle at which robot is considered "down"
-define(MAX_ANGLE, 25.0).

%Coefficient for the complementary filter
-define(COEF_FILTER, 0.667).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Internal functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

complem_angle({Dt, Ax, Az, Gy, Gy0, K, Angle_Complem, Angle_Rate}) ->

    % Compute the new angular rate using a low-pass filter
    Angle_Rate_New = (Gy - Gy0) * ?COEF_FILTER + Angle_Rate * (1 - ?COEF_FILTER),

    % Compute the angle increment from the gyroscope data
    Delta_Gyr = Angle_Rate_New * Dt,

    % Compute the absolute angle from the accelerometer data
    Angle_Acc = math:atan(Az / (-Ax)) * 180 / math:pi(),

    % Apply the complementary filter to combine gyroscope and accelerometer data
    Angle_Complem_New = (Angle_Complem + Delta_Gyr) * K + Angle_Acc * (1 - K), 
    
    % Return the updated complementary angle and angular rate
    {Angle_Complem_New, Angle_Rate_New}.

% Selects the angle based on the provided switch.
% If the switch is true, the Kalman filter angle is selected.
% Otherwise, the complementary filter angle is selected.
select_angle(Switch, Angle_Kalman, Angle_Complem) ->
    if
        Switch -> 
            Angle = Angle_Kalman;  % Use Kalman filter angle
        true ->
            Angle = Angle_Complem  % Use complementary filter angle
    end,
    Angle.

% Computes the speed reference based on the forward and backward inputs.
% If the forward input is true, the speed reference is set to the maximum forward speed.
% If the backward input is true, the speed reference is set to the maximum backward speed.
% If neither input is true, the speed reference is set to zero.
speed_ref(Forward, Backward) ->
    if
        Forward ->
            Adv_V_Goal = ?ADV_V_MAX;  % Set speed to maximum forward speed
        Backward ->
            Adv_V_Goal = - ?ADV_V_MAX;  % Set speed to maximum backward speed
        true ->
            Adv_V_Goal = 0.0  % Set speed to zero
    end,
    Adv_V_Goal.

%% @doc Determines the target turning velocity (`Turn_V_Goal`) based on the input flags `Left` and `Right`.
%%      - If `Right` is true, the target turning velocity is set to the maximum turning velocity (`?TURN_V_MAX`).
%%      - If `Left` is true, the target turning velocity is set to the negative of the maximum turning velocity (`-?TURN_V_MAX`).
%%      - If neither `Left` nor `Right` is true, the target turning velocity is set to 0.0.
%%
%% @param Left A boolean flag indicating whether to turn left.
%% @param Right A boolean flag indicating whether to turn right.
%% @return The target turning velocity (`Turn_V_Goal`) as a float.
turn_ref(Left, Right) ->
    if
        Right ->
            Turn_V_Goal = ?TURN_V_MAX;
        Left ->
            Turn_V_Goal = - ?TURN_V_MAX;
        true ->
            Turn_V_Goal = 0.0
    end,
    Turn_V_Goal.


%% @doc Computes the frequency and its mean over a series of time intervals.
%%      - If the counter `N` reaches 100, it resets the counter and frequency accumulator.
%%      - Otherwise, it updates the frequency and mean frequency using the provided time interval `Dt`.
%%
%% @param Dt The time interval in seconds.
%% @param N The current counter for the number of intervals.
%% @param Freq The current frequency accumulator.
%% @param Mean_Freq The current mean frequency.
%% @return A tuple `{N_New, Freq_New, Mean_Freq_New}` containing the updated counter, frequency, and mean frequency.
frequency_computation(Dt, N, Freq, Mean_Freq) ->
    if
        N == ?FREQ_WINDOW ->
            {0, 0, Freq};  % Reset window
        true ->
            {N + 1, ((Freq * N) + (1 / Dt)) / (N + 1), Mean_Freq}
    end.

%% @doc Pauses execution for `T` milliseconds.
wait(T) -> wait_help(erlang:system_time() / 1.0e6, T).

wait_help(Tnow, Tend) when Tnow >= Tend -> ok;
wait_help(_, Tend) -> wait_help(erlang:system_time() / 1.0e6, Tend).

%% @doc Manages LED flickering pattern during active logging.
%% @param Logging - Boolean flag indicating whether logging is active.
%% @param N - The current loop iteration count.
%% @return - The updated LED state based on the logging status and loop count.
flicker_led(true, N) ->
    if
        N rem 9 < 4 ->
            [grisp_led:color(L, yellow) || L <-[1,2]];
        true ->
            [grisp_led:color(L, {0,0,0}) || L <-[1,2]]
    end;

flicker_led(false, _) ->
    ok.


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Grisps identification
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% @doc Reads the jumper GPIO pins to identify the GRiSP board.
%% @return - A tuple containing the GRiSP ID based on the jumper settings.
get_grisp_id() ->
    %% Jumper GPIO pins
    JMP1 = grisp_gpio:open(jumper_1, #{mode => input}),
    JMP2 = grisp_gpio:open(jumper_2, #{mode => input}),
    JMP3 = grisp_gpio:open(jumper_3, #{mode => input}),
    JMP4 = grisp_gpio:open(jumper_4, #{mode => input}),
    JMP5 = grisp_gpio:open(jumper_5, #{mode => input}),

    %% Read the jumper GPIO pins
    V1 = grisp_gpio:get(JMP1),
    V2 = grisp_gpio:get(JMP2),
    V3 = grisp_gpio:get(JMP3),
    V4 = grisp_gpio:get(JMP4),
    V5 = grisp_gpio:get(JMP5),

    % Convert the jumper values to a single byte
    SUM = (V1) + (V2 bsl 1) + (V3 bsl 2) + (V4 bsl 3) + (V5 bsl 4),
    {ok, SUM}.

