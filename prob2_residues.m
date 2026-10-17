clear; close all; clc;

%% Plant
Gp = tf(1, [1 -1.52 1.5]);

%% Selected PID controller
K = 434.7576;
z1 = 0.5;
z2 = 1.0;
p = 10;

Gc = K * tf( ...
    conv([1 z1], [1 z2]), ...
    [1 p 0]);

%% Closed-loop transfer function
Gcl = feedback(Gc * Gp, 1);

disp('Closed-loop transfer function:');
Gcl

disp('Closed-loop poles:');
disp(pole(Gcl));

%% Step response transfer function
% Y(s) = Gcl(s) / s

Y = Gcl * tf(1, [1 0]);

[num, den] = tfdata(Y, 'v');

%% Partial fraction expansion
[r, poles, direct] = residue(num, den);

% Identify pole at origin (steady-state term)
tol = 1e-7;

ss_idx = abs(poles) < tol;
transient_idx = ~ss_idx;

% Extract steady-state contribution
y_ss = real(sum(r(ss_idx)));

% Extract transient poles and residues
poles = poles(transient_idx);
r = r(transient_idx);

% Sort by descending real part
[~, idx] = sort(real(poles), 'descend');

poles = poles(idx);
r = r(idx);

%% Display residues
fprintf('\n========== TRANSIENT POLE RESIDUES ==========\n');

for i = 1:length(poles)
    fprintf(['Pole = %9.4f %+9.4fj | ' ...
        'Residue = %9.5f %+9.5fj | ' ...
        '|Residue| = %.5f\n'], ...
        real(poles(i)), imag(poles(i)), ...
        real(r(i)), imag(r(i)), abs(r(i)));
end

%% Modal contributions
t = linspace(0, 6, 6001);

modes = zeros(length(poles), length(t));

for i = 1:length(poles)
    modes(i,:) = r(i) * exp(poles(i)*t);
end

% Identify real and complex modes
real_idx = abs(imag(poles)) < tol;
complex_idx = ~real_idx;

% Sum complex conjugate contributions
complex_contribution = real(sum(modes(complex_idx,:), 1));

% Sum real-pole contributions
real_contribution = real(sum(modes(real_idx,:), 1));

% Full reconstructed response
y_reconstructed = y_ss + ...
    complex_contribution + real_contribution;

% Complex-pair approximation
y_complex_only = y_ss + complex_contribution;

%% Compare responses
[y_actual, t_actual] = step(Gcl, t);
y_actual = squeeze(y_actual);

figure;
hold on;
grid on;

plot(t_actual, y_actual, ...
    'k', 'LineWidth', 1.8, ...
    'DisplayName', 'Actual step response');

plot(t, y_reconstructed, 'y:', ...
    'LineWidth', 1.5, ...
    'DisplayName', 'Full residue reconstruction');

plot(t, y_complex_only, 'r--', ...
    'LineWidth', 1.4, ...
    'DisplayName', 'Complex pair + steady-state');

yline(1.02, 'k:');
yline(0.98, 'k:');

xlabel('Time (s)');
ylabel('Output y(t)');

title('Full Step Response vs Complex-Pole Approximation');
legend('Location', 'best');

%% Quantify approximation error
reconstruction_error = max(abs(y_actual(:).' - y_reconstructed));

complex_approx_error = max(abs(y_actual(:).' - y_complex_only));

fprintf('\nMaximum reconstruction error: %.3e\n', ...
    reconstruction_error);

fprintf('Maximum complex-pair approximation error: %.5f\n', ...
    complex_approx_error);