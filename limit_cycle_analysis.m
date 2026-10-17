
clear; close all; clc;

%% Parameters
a = 3;
b = 5;
c = 1;

alpha1 = 1 + (a + 10)/25;
alpha2 = alpha1/3;

beta1 = 1 + b/10;
beta2 = c/25;
beta3 = (a + b)/100;

%% Nonlinear differential equation
odefun = @(t, y) [
    y(2);
    (alpha1 - alpha2*y(1)^2)*y(2) ...
    - beta1*y(1) ...
    - beta2*y(1)^3 ...
    - beta3*y(1)^5
    ];

%% Initial conditions
ICs = [
    0.5  0;
    3.29 0;
    5.0  0;
    0.0  0.1;
    0.0  1;
    0.0  8
    ];

%% Simulation settings
tspan = [0 100];

opts = odeset('RelTol', 1e-9, ...
    'AbsTol', 1e-11, ...
    'MaxStep', 0.01);

%% Analyse each initial condition
n = size(ICs,1);

results = zeros(n,5);

for i = 1:n

    [t,y] = ode45(odefun, tspan, ICs(i,:), opts);

    % Analyse final 50% of simulation
    idx = t >= 50;

    t_ss = t(idx);
    x_ss = y(idx,1);

    % Find positive and negative peaks
    [pks_max, t_max] = findpeaks(x_ss, t_ss);
    [pks_min_neg, ~] = findpeaks(-x_ss, t_ss);

    % Calculate steady-state quantities
    xmax = mean(pks_max);
    xmin = -mean(pks_min_neg);

    A = (xmax - xmin)/2;

    T = mean(diff(t_max));
    f = 1/T;

    results(i,:) = [xmax, xmin, A, T, f];
end

%% Display results
resultsTable = array2table( ...
    [ICs, results], ...
    'VariableNames', { ...
    'x0', 'xdot0', 'xmax', 'xmin', ...
    'Amplitude', 'Period', 'Frequency'});

disp(resultsTable);
