clear;
close all;
clc;

%% Plant

a11 = -1.3;
a12 = -5;
a21 = 0.35;
a22 = -1.31;
a23 = 0.003;
b1 = 0.153;
b2 = 1.351;

num = [b2 + b1*a23, ...
       -b2*a11 + b1*a21];

den = [1, ...
       -(a11 + a22) - a12*a23, ...
       a11*a22 - a12*a21, ...
       0];

G = tf(num, den);

%% PD Controller with zero at 2.5 and pole at 10
zc = 2.5;
pc = 10;

G_C = tf([1 zc], [1 pc]);

G_CL = G_C * G;

figure;
rlocus(G_CL);
grid on;
hold on;

sgrid([0.4 0.5 0.6 0.7 0.8], []);

title("Closed Loop Poles of System")

%% Select root-locus point

[K, cl_poles] = rlocfind(G_CL, -3.01175);

%% Calculate damping ratios

zeta = -real(cl_poles) ./ abs(cl_poles);

fprintf('\nSelected controller:\n');
fprintf('zc = %.2f\n', zc);
fprintf('pc = %.2f\n', pc);
fprintf('K  = %.3f\n\n', K);

fprintf('Closed-loop poles and damping ratios:\n');

for i = 1:length(cl_poles)

    fprintf('Pole: %8.4f %+8.4fj    zeta = %.4f\n', ...
        real(cl_poles(i)), ...
        imag(cl_poles(i)), ...
        zeta(i));

end


%% Mark selected closed-loop poles on root locus

plot(real(cl_poles), imag(cl_poles), 'ro', ...
    'MarkerSize', 8, ...
    'LineWidth', 1.5);

% Label complex poles with damping ratio
for i = 1:length(cl_poles)

    if abs(imag(cl_poles(i))) > 1e-6

        text(real(cl_poles(i)) + 0.2, ...
             imag(cl_poles(i)), ...
             sprintf('\\zeta = %.3f', zeta(i)), ...
             'FontSize', 10);

    end

end


%% Build closed-loop system

T = feedback(K * G_CL, 1);
disp('Step response:')
disp(stepinfo(T))

figure;
step(T);
grid on;

title(sprintf( ...
    'Step Response: z=%.1f, p=%.1f, K=%.2f', ...
    zc, pc, K));


%% Specification checks

fprintf('\n--- Specification checks ---\n');

fprintf('Stable: %d\n', isstable(T));
fprintf('DC gain: %.4f\n', dcgain(T));

info = stepinfo(T);

fprintf('Settling time: %.4f s\n', info.SettlingTime);
fprintf('Overshoot: %.2f %%\n', info.Overshoot);

fprintf('\nClosed-loop poles:\n');
disp(pole(T));

fprintf('Closed-loop zeros:\n');
disp(zero(T));

figure;
pzmap(T);
grid on;
title('Closed-loop Pole-Zero Map');
