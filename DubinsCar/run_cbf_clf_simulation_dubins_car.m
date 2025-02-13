clc;clear; close all;
%%
dt = 0.02;
sim_t = 25;
x0 = [0;0;pi/2];

params.v = 0.05; % velocity
params.u_max = 0.5; % max yaw rate (left)
params.u_min = -0.5; % min yaw rate (right)

% Obstacle position
% params.xo1 = 0.35;
% params.yo1 = 0.3;
% params.xo2 = 0.35;
% params.yo2 = 0.3;
params.xo1 = 0.5;
params.yo1 = 0.3;
params.xo2 = 0.2;
params.yo2 = 0.8;

% Obstacle radius
params.d = 0.1;

params.cbf_gamma0 = 1;
% Desired target point
params.xd = 0.6;
params.yd = 1;

params.clf.rate = 500;
params.weight.slack = 20;

params.cbf.rate = 1;

dubins = DubinsCar(params);

odeFun = @dubins.dynamics;
controller = @dubins.ctrlCbfClfQp;
odeSolver = @ode45;

total_k = ceil(sim_t / dt);
x = x0;
t = 0;   
% initialize traces.
xs = zeros(total_k, dubins.xdim);
ts = zeros(total_k, 1);
us = zeros(total_k-1, 1);
Vs = zeros(total_k-1, 1);
hs = zeros(total_k-1, 2);
xs(1, :) = x0';
ts(1) = t;
for k = 1:total_k-1
%     t
    % Determine control input.
    % dV_hat: analytic Vdot based on model.
    [u, slack, h, V] = controller(x);        
    us(k, :) = u';
    hs(k, :) = h';
    Vs(k) = V;

    % Run one time step propagation.
    [ts_temp, xs_temp] = odeSolver(@(t, s) odeFun(t, s, u), [t t+dt], x);
    x = xs_temp(end, :)';

    xs(k+1, :) = x';
    ts(k+1) = ts_temp(end);
    t = t + dt;
    
    disToGoal = sqrt((x(1) - params.xd)^2 + (x(2) - params.yd)^2)
    if disToGoal < 0.01
        break;
    end
end
%%
plot_results(ts, xs, us, hs, [params.xo1, params.yo1; params.xo2, params.yo2], params.d)

%%
function plot_results(t, xs, us, hs, p_o, r_o)

figure
subplot(3,1,1)
plot(t, xs(:,1))
xlabel('t')
ylabel('x [m]')

subplot(3,1,2)
plot(t, xs(:,2))
xlabel('t')
ylabel('y [m]')

subplot(3,1,3)
plot(t, xs(:,3))
xlabel('t')
ylabel('theta [rad]')


figure
plot(t(1:end-1), us)
xlabel('t')
ylabel('u [rad/s]')


lim_min = min(min(xs(:, 1)), min(xs(:, 2)));
lim_max = max(max(xs(:, 1)), max(xs(:, 2)));
% lim_min = min([lim_min, p_o(1)-r_o, p_o(2)-r_o]);
% lim_max = max([lim_max, p_o(1)+r_o, p_o(2)+r_o]);

figure
plot(xs(:, 1), xs(:, 2));
hold on
plot(0.6, 1, '*r')
hold on
draw_circle(p_o(1, :), r_o);
hold on
draw_circle(p_o(2, :), r_o);
hold off
legend("Path", "Goal", "Ob1", "Ob2")

xlim([lim_min, lim_max]);
ylim([lim_min, lim_max]);
xlabel('x [m]')
ylabel('y [m]')

figure
plot(t(1:end-1), hs)
xlabel('t')
ylabel('cbf h(s)');

end

function h = draw_circle(center,r)
hold on
th = 0:pi/50:2*pi;
xunit = r * cos(th) + center(1);
yunit = r * sin(th) + center(2);
h = plot(xunit, yunit);
hold off

end