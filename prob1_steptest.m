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

%% Build closed-loop system

T = feedback(K * G_CL, 1);
disp('Step response:')
disp(stepinfo(T))

figure;
step(T);
grid on;

title(sprintf( ...
    'Step Response of Closed Loop System G_{cl}(s)'));


%% Specification checks

fprintf('\n--- Specification checks ---\n');

stable = "";
if isstable(T)
    stable = "True";
else
    stable = "False";
end

fprintf('Stable: %s\n', stable);
fprintf('DC gain: %.4f\n', dcgain(T));

info = stepinfo(T);

fprintf('Settling time: %.4f s\n', info.SettlingTime);
fprintf('Overshoot: %.2f %%\n', info.Overshoot);

fprintf('\nClosed-loop poles:\n');
disp(pole(T));

fprintf('Closed-loop zeros:\n');
disp(zero(T));