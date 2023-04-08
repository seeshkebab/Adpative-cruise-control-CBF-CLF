close all;
clear all;
% Init state.
x0 = [0; 0; 4; 0];

%% define system

syms p_x v_x p_y v_y;
x = [p_x; v_x; p_y; v_y];

A = zeros(4);
A(1, 2) = 1;
A(3, 4) = 1;
B = [0 0; 1 0; 0 0; 0 1];

f = A * x;
g = B;

%% CLF and CBF+
clf = defineClf1(x);
cbf = defineCbf(x);







% % Target position
% p_d = [10; 0];
% % obstacle center
% p_o = [5; 2];
% % obstacle radius.
% r_o = 2; 
% 
% dt = 0.02;
% sim_t = 30;
% 
% cbf_gamma0 = 1;
% 
% u_max = 7;
% u_min  = -7;
% 
% clf.rate = 0.7;
% cbf.rate = 3;
% 
% weight.slack = 1;
% weight.input = 5;
% 
% dynsys = DoubleIntegrator2D(params);
% 
% odeFun = @dynsys.dynamics;
% controller = @dynsys.ctrlCbfClfQp;
% % odeSolver = @ode45;
% 
% total_k = ceil(sim_t / dt);
% x = x0;
% t = 0;   
% % initialize traces.
% xs = zeros(total_k, dynsys.xdim);
% ts = zeros(total_k, 1);
% us = zeros(total_k-1, dynsys.udim);
% hs = zeros(total_k-1, 1);
% Vs = zeros(total_k-1, 1);
% xs(1, :) = x0';
% ts(1) = t;
% u_prev = [0;0];
% for k = 1:total_k-1
%     t
%     % Determine control input.
%     % dV_hat: analytic Vdot based on model.
%     [u, slack, h, V] = controller(x);        
% %     [u, slack, h, V] = controller(s, u_prev); % optimizing the difference between the previous timestep.       
%     us(k, :) = u';
%     hs(k) = h;
%     Vs(k) = V;
% 
%     % Run one time step propagation.
%     [ts_temp, xs_temp] = ode45(@(t, s) odeFun(t, s, u), [t t+dt], x);
%     x = xs_temp(end, :)';
% 
%     xs(k+1, :) = x';
%     ts(k+1) = ts_temp(end);
%     u_prev = u;
%     t = t + dt;
% end

% plot_results(ts, xs, us, p_o, r_o)
function clf = defineClf1(symbolic_state)
            x = symbolic_state;
            p_d = [10; 0];

            A = zeros(4);
            A(1, 2) = 1;
            A(3, 4) = 1;
            B = [0 0; 1 0; 0 0; 0 1];
            Q = eye(size(A));
            R = eye(size(B,2));
            [~,P] = lqr(A,B,Q,R);
            e = x - [p_d(1); 0; p_d(2); 0];
            clf = e' * P * e;        
end

function cbf = defineCbf(symbolic_state)
            p_o = [5; 2];
            r_o = 2; 
            x = symbolic_state;
            p_o = p_o; % position of the obstacle.
            r_o = r_o; % radius of the obstacle.
            distance = (x(1) - p_o(1))^2 + (x(3) - p_o(2))^2 - r_o^2;
            derivDistance = 2*(x(1)-p_o(1))*x(2) + 2*(x(3)-p_o(2))*x(4);
            cbf = derivDistance + cbf_gamma0 * distance;
end