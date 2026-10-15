clear; close all; clc;

%% Parameters (replace with your values)
a = 3;
b = 5;
c = 1;

alpha1 = 1 + (a + 10)/25;
alpha2 = alpha1/3;

beta1 = 1 + b/10;
beta2 = c/25;
beta3 = (a + b)/100;

%% Differential equation
odefun = @(t, y) [
    y(2);
    (alpha1 - alpha2*y(1)^2)*y(2) ...
    - beta1*y(1) ...
    - beta2*y(1)^3 ...
    - beta3*y(1)^5
    ];

%% Simulation
tspan = [0 40];

% Initial conditions
ICs_x = [
    0.5  0;
    3.29 0;
    5.0  0;
    ];

ICs_xdot = [
    0 0.1;
    0  1;
    0  8;
    ];

%% Figure 1: Varying initial displacement
figure;
hold on;
grid on;

for i = 1:size(ICs_x,1)
    [t,y] = ode45(odefun, tspan, ICs_x(i,:));

    plot(t, y(:,1), 'LineWidth', 1.2, ...
        'DisplayName', sprintf('x(0) = %.2f', ICs_x(i,1)));
end

xlabel('Time (s)');
ylabel('x(t)');
title('Response for Varying Initial $x(0)$', 'Interpreter', 'latex');
legend('Location', 'best');
xlim(tspan);
ylim([-6 6]);

exportgraphics(gcf, 'varying_displacement.pdf', ...
    'ContentType', 'vector');


%% Figure 2: Varying initial velocity
figure;
hold on;
grid on;

for i = 1:size(ICs_xdot,1)
    [t,y] = ode45(odefun, tspan, ICs_xdot(i,:));

    plot(t, y(:,1), 'LineWidth', 1.2, ...
        'DisplayName', sprintf('dx/dt(0) = %.2f', ICs_xdot(i,2)));
end

xlabel('Time (s)');
ylabel('x(t)');
title('Response for Varying Initial $\dot{x}(0)$', 'Interpreter', 'latex');
legend('Location', 'best');
xlim(tspan);
ylim([-6 6]);

exportgraphics(gcf, 'varying_velocity.pdf', ...
    'ContentType', 'vector');
