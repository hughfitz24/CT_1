clear; close all; clc;

%% Plant
Gp = tf(1, [1 -1.52 1.5]);

%% Design specifications
Ts_max = 2.04;
PO_max = 80;
SSE_tol = 1e-6;

% Approximate dominant pole requirement
sigma_req = -4/Ts_max;

% Minimum damping ratio for overshoot
Mp = PO_max/100;
zeta_min = -log(Mp)/sqrt(pi^2 + log(Mp)^2);

%% Controller parameters
Td = 0.1;
p = 1/Td;

z1 = 0.5;
z2 = 1.0;

% Controller without gain
Gc_shape = tf( ...
    conv([1 z1], [1 z2]), ...
    [1 p 0]);

% Open-loop system
L = Gc_shape * Gp;

%% Root locus
figure;
rlocus(L);
hold on;
grid on;

% Settling-time boundary
xline(sigma_req, 'k--', 'LineWidth', 1.2, 'Color', "b");

% Damping ratio requirement
sgrid();

xlim([-8 3]);
ylim([-20 20]);

title(sprintf('Root Locus: z_1 = %.2f, z_2 = %.2f', ...
    z1, z2));

%% Interactive gain selection
try_again = true;

while try_again

    fprintf('\nSelect a point on the root locus.\n');

    % Select gain interactively
    [K, selected_poles] = rlocfind(L);

    fprintf('\nSelected gain K = %.4f\n', K);

    %% Construct closed-loop system
    Gc = K * Gc_shape;
    Gcl = feedback(Gc * Gp, 1);

    %% Analyse response
    cl_poles = pole(Gcl);
    info = stepinfo(Gcl);
    dc_gain = dcgain(Gcl);

    %% Specification verification
    stable = isstable(Gcl);
    zero_sse = stable && abs(dc_gain - 1) < SSE_tol;
    settling_ok = stable && info.SettlingTime <= Ts_max;
    overshoot_ok = stable && info.Overshoot <= PO_max;

    all_specs = stable && zero_sse && ...
                settling_ok && overshoot_ok;

    %% Results
    fprintf('\n========== CLOSED-LOOP POLES ==========\n');
    disp(cl_poles);

    fprintf('\n========== SPECIFICATION CHECK ==========\n');
    fprintf('Stable:              %d\n', stable);
    fprintf('Zero step SSE:       %d\n', zero_sse);
    fprintf('DC gain:             %.6f\n', dc_gain);
    fprintf('Settling time:       %.4f s (limit %.2f s)\n', ...
        info.SettlingTime, Ts_max);
    fprintf('Overshoot:           %.2f %% (limit %.0f %%)\n', ...
        info.Overshoot, PO_max);

    if all_specs
        fprintf('\nALL SPECIFICATIONS SATISFIED\n');
    else
        fprintf('\nONE OR MORE SPECIFICATIONS NOT SATISFIED\n');
    end

    %% Step response
    figure;
    step(Gcl, 6);
    hold on;
    grid on;

    xline(Ts_max, 'r--', 'Maximum Settling Time');

    % Settling band
    yline(1.02, 'k:', 'Upper 2% Band');
    yline(0.98, 'k:', 'Lower 2% Band');

    % Overshoot limit
    yline(1 + PO_max/100, 'r:', 'Overshoot Limit');

    title(sprintf( ...
        'Closed-Loop Step Response: K = %.4f', K));

    xlabel('Time (s)');
    ylabel('Output x(t)');

    %% Ask whether to try another gain
    choice = input('\nTry another gain? (y/n): ', 's');

    try_again = strcmpi(strtrim(choice), 'y');

end

%% Final controller
fprintf('\n========== FINAL CONTROLLER ==========\n');

Gc