%% Problem 2: understand integral gain and the PI zero
% Requires Control System Toolbox. Run the whole script or individual sections.
% Linearised plant from Section 2 of main.tex; unity negative feedback.
% C(s) = Kp + Ki/s = Kp*(s+zc)/s, where Ki = Kp*zc.
% The zero is at s = -zc (zc is a positive magnitude, in rad/s).
% Changing Ki at fixed Kp ALSO moves the zero. They are not independent.
clear; close all; clc;

assert(exist('tf', 'file') ~= 0, 'Control System Toolbox is required.');
s = tf('s');
Gp = 1/(s^2 - 1.52*s + 1.5);

%% Editable experiments
Kp_fixed = 5;
Ki_values = [0 0.1 0.5 2 5 10];
zc_values = [0.05 0.2 0.76 1.5 3 6];
zc_fixed = 0.5;
Kp_values = [0.1 1 5 20 100];
t = linspace(0, 6, 3001); % finite window: these responses do not settle

fprintf('Plant: Gp(s) = 1/(s^2 - 1.52s + 1.5)\n');
fprintf('PI: C(s) = Kp + Ki/s = Kp(s+zc)/s; zc = Ki/Kp.\n\n');
fprintf('For nonzero Ki the characteristic polynomial is:\n');
fprintf('s^3 - 1.52s^2 + (1.5+Kp)s + Ki = 0.\n');
fprintf('Sum of the three poles = +1.52, independent of both gains.\n');
fprintf('At least one pole has positive real part: PI cannot stabilise this plant.\n');
fprintf('For positive Kp and Ki, the Routh first column has signs +,-,+,+:\n');
fprintf('there are TWO right-half-plane poles.\n\n');
fprintf('An integrator supplies an OPEN-loop pole at zero.\n');
fprintf('Zero step error follows only if the closed loop is stable.\n');
fprintf('A formal DC gain of 1 here does not imply a final value of 1.\n\n');

%% Experiment 1: increase integral action while holding Kp fixed
% Ki=0 is implemented as P only, avoiding an artificial cancelled integrator.
figure('Name', 'Integral gain at fixed proportional gain');
tiledlayout(1, 2);
axp = nexttile; hold(axp, 'on'); grid(axp, 'on');
xline(axp, 0, 'k:', 'HandleVisibility', 'off');
xlabel(axp, 'Real part (1/s)'); ylabel(axp, 'Imaginary part (rad/s)');
title(axp, sprintf('Closed-loop poles: Kp = %g', Kp_fixed));
axt = nexttile; hold(axt, 'on'); grid(axt, 'on');
yline(axt, 1, 'k:', 'HandleVisibility', 'off');
xlabel(axt, 'Time (s)'); ylabel(axt, 'Output x(t)');
title(axt, 'Unit step: finite window, no steady state');
colours = lines(numel(Ki_values));
summary = zeros(numel(Ki_values), 5);
for n = 1:numel(Ki_values)
    Ki = Ki_values(n);
    if Ki == 0
        C = Kp_fixed;
        label = sprintf('P only: Ki = 0');
    else
        C = Kp_fixed + Ki/s;
        label = sprintf('Ki = %g, zero = -%g', Ki, Ki/Kp_fixed);
    end
    T = feedback(C*Gp, 1);
    p = pole(T);
    y = step(T, t);
    plot(axp, real(p), imag(p), 'x', 'Color', colours(n,:), ...
        'MarkerSize', 9, 'LineWidth', 1.5, 'DisplayName', label);
    plot(axt, t, y, 'Color', colours(n,:), 'LineWidth', 1.2, ...
        'DisplayName', label);
    summary(n,:) = [Kp_fixed Ki -Ki/Kp_fixed max(real(p)) isstable(T)];
end
legend(axp, 'Location', 'best'); legend(axt, 'Location', 'best');
results = array2table(summary, 'VariableNames', ...
    {'Kp', 'Ki', 'ZeroLocation', 'RightmostPoleReal', 'Stable'});
fprintf('Experiment 1 (the Ki=0 row has no controller zero):\n');
disp(results);

%% Experiment 2: move the zero at fixed Kp and inspect the root locus
% Each locus sweeps K in C(s)=K*(s+zc)/s. A red cross marks K=Kp_fixed.
% Moving the zero changes Ki=Kp*zc; it does not supply derivative damping.
figure('Name', 'PI zero placement and root locus');
tiledlayout(2, 3);
for n = 1:numel(zc_values)
    zc = zc_values(n);
    nexttile;
    rlocus((s+zc)/s * Gp); hold on; grid on;
    p = pole(feedback(Kp_fixed*(s+zc)/s * Gp, 1));
    plot(real(p), imag(p), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
    xline(0, 'k:');
    title(sprintf('Zero = -%g; red: Kp=%g, Ki=%g', ...
        zc, Kp_fixed, Kp_fixed*zc));
end

%% Experiment 3: increase total PI gain while holding its zero fixed
% Both Kp and Ki increase proportionally. This separates gain scaling from
% moving the zero, unlike Experiment 1.
figure('Name', 'Gain scaling at fixed PI zero');
hold on; grid on;
for n = 1:numel(Kp_values)
    Kp = Kp_values(n);
    T = feedback(Kp*(s+zc_fixed)/s * Gp, 1);
    plot(t, step(T, t), 'LineWidth', 1.2, 'DisplayName', ...
        sprintf('Kp=%g, Ki=%g', Kp, Kp*zc_fixed));
end
yline(1, 'k:', 'HandleVisibility', 'off');
xlabel('Time (s)'); ylabel('Output x(t)');
title(sprintf('Zero fixed at -%g: gain cannot remove instability', zc_fixed));
legend('Location', 'best');

%% Interpretation and a next design direction
% For Kp>0, Ki>0, one real pole is negative and the complex pair is unstable.
% With Ki approaching zero, a slow pole approaches the origin and the other
% two poles approach the unstable P-only poles. Adding I does not add damping.
%
% Ideal PID would instead give:
% s^3 + (Kd-1.52)*s^2 + (1.5+Kp)*s + Ki.
% The cubic Routh conditions are:
% Kd>1.52, Kp>-1.5, Ki>0, (Kd-1.52)*(1.5+Kp)>Ki.
% Thus, at fixed Kp and Kd, too much integral gain can destabilise even PID.
% These are stability conditions, not guarantees of settling time/overshoot.
% A practical derivative filter changes the polynomial and needs rechecking.
% All experiments here use the local linear model, not the nonlinear ODE.
fprintf('Next topology: derivative action can offset the negative damping.\n');
fprintf('For ideal PID: Kd>1.52 and 0<Ki<(Kd-1.52)*(1.5+Kp), Kp>-1.5.\n');
fprintf('Settling time and overshoot are undefined for the unstable PI designs.\n');
