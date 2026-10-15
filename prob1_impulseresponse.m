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

G_C = 84.019*tf([1 zc], [1 pc]);



%% Compare open-loop and closed-loop disturbance response

Gd_open = G;
Gd_closed = feedback(G, G_C);

t = 0:0.001:10;


[y_open, t] = impulse(Gd_open, t);
[y_closed, ~] = impulse(Gd_closed, t);

figure;
plot(t, y_closed, 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Pitch-angle deviation');
title('Disturbance Response of Closed Loop System with PD Controller');

figure;
plot(t, y_open, 'LineWidth', 1.5);
hold on;
plot(t, y_closed, 'LineWidth', 1.5);
grid on;

xlabel('Time (s)');
ylabel('Pitch-angle deviation');
title('Disturbance Response Comparison');
legend('Uncontrolled plant', 'PD-controlled system');

% Peak deviations
max_open = max(abs(y_open));
max_closed = max(abs(y_closed));

fprintf('Open-loop maximum: %.5f\n', max_open);
fprintf('Closed-loop maximum: %.5f\n', max_closed);
fprintf('Reduction: %.2f %%\n', ...
    100 * (1 - max_closed/max_open));

%% Recovery time

tol = 0.02 * max_closed;

idx = find(abs(y_closed) > tol, 1, 'last');

if isempty(idx)
    T_recovery = 0;
elseif idx == length(t)
    T_recovery = NaN;  % Not settled within simulation
else
    T_recovery = t(idx + 1);
end

fprintf('Recovery time: %.4f s\n', T_recovery);

%% Verify disturbance rejection

disp('Closed-loop disturbance transfer function poles:');
disp(pole(Gd_closed));

if isstable(Gd_closed)
    fprintf('Closed-loop system is asymptotically stable.\n');
    fprintf('Impulse disturbance response converges to zero.\n');
else
    fprintf('Closed-loop system is not asymptotically stable.\n');
end