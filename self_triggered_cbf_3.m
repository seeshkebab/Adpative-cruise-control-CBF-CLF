clear all; 
clc;
A = [0 1; 0 0];
B = [0; 1];
% u = zeros(1,N); % Initialize u with zeros

% Barrier certificate
h = @(x) 1 - x(1)^2 - x(2)^2;

% Control Lyapunov function
V = @(x) x(1)^2 + x(2)^2;

lambda = 1;     % CBF parameter
k = 1;          % CLF parameter
T = 0.1;        % Sampling time
x0 = [1; 1];    % Initial condition

tspan = 0:T:10;                  % Time vector
x = zeros(2, length(tspan));     % State vector
x(:, 1) = x0;                   % Initial state
u = zeros(1, length(tspan)-1);   % Control input
z = h(x0);                      % Barrier value
v = V(x0);                      % Lyapunov value

for i = 1:length(tspan)-1
    if z >= 0
        u(i) = -k*x(:,i);       % CLF controller
        z = h(x(:,i+1));        % Update barrier value
    else
        u(i) = -lambda*z*x(:,i);% CBF controller
        z = h(x(:,i));          % Keep previous barrier value
    end
    x(:, i+1) = x(:, i) + T*A*x(:, i) + T*B*u(i);    % Update state
    v = V(x(:,i+1));                                % Update Lyapunov value
end


subplot(2, 1, 1)
plot(tspan, x(1, :), tspan, x(2, :))
legend('x_1', 'x_2')
xlabel('Time')
ylabel('State')

subplot(2, 1, 2)
plot(tspan(1:end-1), u)
xlabel('Time')
ylabel('Control input')