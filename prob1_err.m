clear; close all; clc;

%% Nominal parameters
a11_nom = -1.3;
a21_nom = 0.35;

% Non-varying parameters
a12 = -5;
a22 = -1.31;
a23 = 0.003;
b1 = 0.153;
b2 = 1.351;

%% PD Controller
K = 84.019;
C = K * tf([1 2.5], [1 10]);

%% Variation of a11 and a21
a11_scale = linspace(0.9, 1.1, 21);
a21_scale = linspace(0.9, 1.1, 21);

N = numel(a11_scale) * numel(a21_scale);
results = nan(N, 4);

%% Stability analysis
n = 0;

for i = 1:numel(a11_scale)
    for j = 1:numel(a21_scale)
        n = n + 1;

        a11 = a11_nom * a11_scale(i);
        a21 = a21_nom * a21_scale(j);

        % Perturbed plant
        G = tf([b2 + b1*a23, -b2*a11 + b1*a21], ...
            [1, -(a11 + a22) - a12*a23, ...
            a11*a22 - a12*a21, 0]);

        % Closed-loop system
        T = feedback(C*G, 1);

        % Closed-loop poles
        p = pole(T);
        maxRealPole = max(real(p));

        % Stability
        stable = isstable(T);

        results(n,:) = [a11_scale(i), a21_scale(j), ...
            stable, maxRealPole];
    end
end

%% Results table
R = array2table(results, 'VariableNames', ...
    {'a11Scale', 'a21Scale', 'Stable', 'MaxRealPole'});

%% Summary
fprintf('Total combinations: %d\n', height(R));
fprintf('Unstable cases: %d\n', sum(R.Stable == 0));

fprintf('Largest real pole component: %.5f\n', ...
    max(R.MaxRealPole));

%% Least stable configuration
[~, worstIdx] = max(R.MaxRealPole);

disp('Least stable parameter combination:');
disp(R(worstIdx,:));