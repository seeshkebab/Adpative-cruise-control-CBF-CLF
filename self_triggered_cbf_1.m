% Example system: double integrator
A = [0 1; 0 0];
B = [0; 1];

% Safety set (obstacle): x2 >= 1
safety_set = @(x) x(2) - 1;

% Control barrier function: -u + x2 - 1 >= 0
cbf = @(x, u) -u + x(2) - 1;

% Initial condition and horizon
x0 = [0; 0];
T = 5;

% Self-triggered control parameters
h = 0.1;  % Sampling time
tau = 0.5;  % Minimum inter-sample time
delta = 0.01;  % Tolerance for CBF constraint

% Simulation loop
t = 0;
x = x0;
u = 0;
while t < T
    % Evaluate CBF and safety set
    s = safety_set(x);
    f = cbf(x, u);
    
    % If CBF constraint is not satisfied, solve optimization problem
    if f < -delta
        cvx_begin
            variable u_opt
            minimize norm(u_opt - u, 2)
            subject to
                cbf(x, u_opt) >= -delta
                abs(u_opt - u) <= 1  % Input constraints
        cvx_end
        u = u_opt;
    end
    
    % Update state using forward Euler integration
    x = x + h * (A * x + B * u);
    
    % Update time and check if inter-sample time has elapsed
    t = t + h;
    if mod(t, tau) < h
        % If inter-sample time has elapsed, reset timer and check CBF constraint again
        t = ceil(t / tau) * tau;
        if cbf(x, u) < -delta
            disp('CBF constraint violated!')
            break;
        end
    end
end

% Plot trajectory
plot(x(1,:), x(2,:))
xlabel('x_1')
ylabel('x_2')
title('Trajectory')