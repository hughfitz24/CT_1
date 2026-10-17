
clear; close all; clc;

%% Plant parameters
alpha1 = 1.52;
alpha2 = 1.52/3;
beta1 = 1.5;
beta2 = 0.04;
beta3 = 0.07;

% Linearised plant
Gp = tf(1, [1 -1.52 1.5]);

%% PID parameters
Kp = 69.37;
Ki = 23.92;
Kd = 40.9;
Td = 0.1;

%% PART 1: LINEARISED STABILITY ANALYSIS

% Controller configurations
C_nominal = tf(Kp) ...
    + tf(Ki,[1 0]) ...
    + tf([Kd 0],[Td 1]);

C_noKp = tf(Ki,[1 0]) ...
    + tf([Kd 0],[Td 1]);

C_noKi = tf(Kp) ...
    + tf([Kd 0],[Td 1]);

C_noKd = tf(Kp) ...
    + tf(Ki,[1 0]);

controllers = {
    C_nominal
    C_noKp
    C_noKi
    C_noKd
};

names = {
    'Nominal PID'
    'Loss of Kp'
    'Loss of Ki'
    'Loss of Kd'
};

fprintf('========== LINEARISED STABILITY ==========\n');

for i = 1:numel(names)

    T = minreal(feedback(controllers{i}*Gp,1),1e-7);
    p = pole(T);

    fprintf('\n%s\n',names{i});
    fprintf('  Max Re(p): %.6f\n',max(real(p)));
    fprintf('  Stable: %d\n',isstable(T));

end

%% Critical derivative gain
maxRealPole = @(kd) max(real(pole( ...
    feedback( ...
        (tf(Kp) + tf(Ki,[1 0]) ...
        + tf([kd 0],[Td 1]))*Gp,1) ...
    )));

KdCritical = fzero(maxRealPole,[0 Kd]);

fprintf('\n========== CRITICAL DERIVATIVE GAIN ==========\n');
fprintf('Nominal Kd: %.4f\n',Kd);
fprintf('Critical Kd: %.4f\n',KdCritical);
fprintf('Allowable reduction: %.2f%%\n', ...
    100*(1-KdCritical/Kd));

%% PART 2: FAILURE SIMULATIONS

tFailure = 0.7;
tEnd = 10;

% States: [x; xdot; integral(error); derivative filter]
y0 = [1; 0; 0; 0];

% Perturbation at failure
velocityKick = 0.1;

opts = odeset('RelTol',1e-8,'AbsTol',1e-10);

scenarioNames = {
    'Nominal PID'
    'Loss of K_d'
    'Complete controller failure'
};

%% Simulate both models
for model = 1:2

    % model = 1: Linearised
    % model = 2: Nonlinear

    if model == 1
        modelName = 'Linearised';
    else
        modelName = 'Nonlinear';
    end

    %% Healthy operation until failure
    [tHealthy,yHealthy] = ode45( ...
        @(t,y) systemODE(y,Kp,Ki,Kd,Td, ...
        alpha1,alpha2,beta1,beta2,beta3, ...
        0,model), ...
        [0 tFailure],y0,opts);

    % State at failure
    yFailure = yHealthy(end,:)';

    % Apply identical perturbation
    yFailure(2) = yFailure(2) + velocityKick;

    %% Simulate failure scenarios
    tResults = cell(3,1);
    yResults = cell(3,1);

    for i = 1:3

        kd = Kd;
        failureMode = 0;

        switch i
            case 1
                % Nominal PID

            case 2
                % Loss of derivative action
                kd = 0;

            case 3
                % Complete controller failure
                failureMode = 1;
        end

        [tPost,yPost] = ode45( ...
            @(t,y) systemODE(y,Kp,Ki,kd,Td, ...
            alpha1,alpha2,beta1,beta2,beta3, ...
            failureMode,model), ...
            [tFailure tEnd],yFailure,opts);

        tResults{i} = [tHealthy; tPost(2:end)];
        yResults{i} = [yHealthy; yPost(2:end,:)];
    end

    %% Plot
    figure;
    hold on; grid on;

    for i = 1:3
        plot(tResults{i},yResults{i}(:,1), ...
            'LineWidth',1.5, ...
            'DisplayName',scenarioNames{i});
    end

    xline(tFailure,'k--', ...
        sprintf('Failure at t = %.1f s',tFailure), ...
        'HandleVisibility','off');

    yline(0,'k:','HandleVisibility','off');

    xlabel('Time (s)');
    ylabel('Displacement x(t)');
    title([modelName ' Plant Response to Controller Failure']);
    legend('Location','best');

    %% Quantify post-failure response
    fprintf('\n========== %s MODEL ==========\n',upper(modelName));

    for i = 1:3

        t = tResults{i};
        x = yResults{i}(:,1);

        idx = t >= (tFailure + 0.8*(tEnd-tFailure));
        xTail = x(idx);

        amplitude = (max(xTail)-min(xTail))/2;

        fprintf('\n%s\n',scenarioNames{i});
        fprintf('  Maximum |x|: %.4f\n', ...
            max(abs(x(t>=tFailure))));
        fprintf('  Final-window amplitude: %.4f\n',amplitude);

    end
end

%% SYSTEM DYNAMICS
function dydt = systemODE(y,Kp,Ki,Kd,Td, ...
    alpha1,alpha2,beta1,beta2,beta3, ...
    failureMode,model)

    % States
    x = y(1);
    xdot = y(2);
    integralError = y(3);
    filterState = y(4);

    % Zero reference
    error = -x;

    % Filtered derivative
    filterDot = (error-filterState)/Td;

    % PID control
    f = Kp*error ...
        + Ki*integralError ...
        + (Kd/Td)*(error-filterState);

    % Complete controller failure
    if failureMode == 1
        f = 0;
    end

    % Plant dynamics
    if model == 1

        % Linearised
        xddot = alpha1*xdot - beta1*x + f;

    else

        % Full nonlinear
        xddot = (alpha1-alpha2*x^2)*xdot ...
            - beta1*x ...
            - beta2*x^3 ...
            - beta3*x^5 ...
            + f;
    end

    dydt = [
        xdot;
        xddot;
        error;
        filterDot
    ];
end
