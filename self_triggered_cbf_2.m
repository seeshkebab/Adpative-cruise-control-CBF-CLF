% Define the double integrator system dynamics
A = [0 1; 0 0];
B = [0; 1];

% Define the desired final state
x_d = [-7; 0];

% Define the self-triggered control update interval
ts = 0.5; % adjust as needed

% Define the initial state and time
x0 = [0; 0];
t0 = 0;

% Set up the quadratic program parameters
Q = eye(2); % weighting matrix for the CLF
R = 1; % weighting matrix for the control input
P = eye(2); % weighting matrix for the CBF
q = zeros(2, 1); % linear term for the CLF
r = 0; % linear term for the control input
p = zeros(2, 1); % linear term for the CBF

% Define the maximum allowed control input
u_max = 1;

% Initialize the simulation variables
x = x0;
t = t0;

while t < 10 % adjust the simulation time as needed
    % Compute the time to the next controller update
    t_next = t + ts;
    
    % Run the simulation until the next controller update
    [t_sim, x_sim] = ode45(@(t, x) double_integrator(t, x, u), [t, t_next], x);
    
    % Update the current time and state
    t = t_sim(end);
    x = x_sim(end, :)';
    
    % Compute the control input using CBF-CLF QP solver
    H = [Q zeros(2, 1); zeros(1, 2) R];
    f = [q; r];
    A_ineq = [-1 0 -p(1); 1 0 -p(1); 0 -1 -p(2); 0 1 -p(2); 0 0 1];
    b_ineq = [-x_d(1) + p(1)*x_d(2) + x(1) - q'*x; x_d(1) + p(1)*x_d(2) - x(1) + q'*x; 
        -x_d(2) + p(2)*x_d(1) + x(2) - u_max; x_d(2) + p(2)*x_d(1) - x(2) + u_max; 
        u_max];
    z = quadprog(H, f, A_ineq, b_ineq);
    u = z(end);
    
    % Apply the control input to the system
    x_dot = A * x + B * u;
    x = x + x_dot * ts;
end


figure;
plot(t_sim, x_sim(:, 1), 'b-', 'LineWidth', 2);
hold on;
plot([t0, t], [x0(1), x(1)], 'r-', 'LineWidth', 2);
xlabel('Time');
ylabel('Position');
legend('Simulated trajectory', 'Controlled trajectory');
title('Double integrator system with self-triggered control');

function x_dot = double_integrator(t, x, u)
% Define the double integrator system dynamics
A = [0 1; 0 0];
B = [0; 1];

x_dot = A * x + B * u;
end