clc; clear; close all;
%%
% dt = 0.02;
dt = 0.02;
sim_t = 25;
x0 = [0; 0; 0];

params.v = 0.1; % velocity
params.r_rob = 0.08;

params.ctrl_input = "w";
% if control input is [w]
params.u_max =  1; % max yaw rate (left)
params.u_min = -1; % min yaw rate (right)

% params.ctrl_input = "v_w";
% if control input is [v; w]
% params.u_max =  [  0.5;  1]; % max yaw rate (left)
% params.u_min =  [ 0.01; -1]; % min yaw rate (right)

% Obstacle position
params.xo = [0.15, 0.35];
params.yo = [0.85, 0.35];

% Obstacle radius
params.d = 0.1;
params.cbf_gamma0 = 0.25;

% Desired target point
params.xd = 0.6;
params.yd = 1;

% params.clf.rate = 500;
params.clf.rate = 5;
params.weight.slack = 50;

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
us = zeros(total_k-1, length(params.u_max));
Vs = zeros(total_k-1, 1);
hs = zeros(total_k-1, length(params.xo));
xs(1, :) = x0';
ts(1) = t;

%%
for k = 1:total_k-1
    t
    % Determine control input.
    % dV_hat: analytic Vdot based on model.
    [u, slack, h, V] = controller(x);        
    us(k, :) = u';
    hs(k, :) = h;
    Vs(k) = V;

    % Run one time step propagation.
    [ts_temp, xs_temp] = odeSolver(@(t, s) odeFun(t, s, u), [t t+dt], x);
    x = xs_temp(end, :)';

    xs(k+1, :) = x';
    ts(k+1) = ts_temp(end);
    t = t + dt
    goalDist = sqrt((x(1) - params.xd)^2 + (x(2) - params.yd)^2);
    if goalDist < 0.05
        break;
    end
end

%%
plot_results(k, ts, xs, us, hs, Vs,[params.xo; params.yo], params.d, x0(1:2), [params.xd, params.yd])

%%
function plot_results(num, t, xs, us, hs, Vs, p_o, r_o, start, goal)

figure
subplot(3,1,1)
plot(t(1:num), xs(1:num, 1))
xlabel('t')
ylabel('x [m]')

subplot(3,1,2)
plot(t(1:num), xs(1:num, 2))
xlabel('t')
ylabel('y [m]')

subplot(3,1,3)
plot(t(1:num), xs(1:num, 3), 'DisplayName', 'theta [rad]')
hold on
plot(t(1:num), us(1:num), 'DisplayName', 'u [rad/s]')
xlabel('t')
ylabel('theta [rad] / u [rad/s]')
legend('Location', 'Best')

figure
plot(xs(1:num, 1), xs(1:num, 2));
hold on
plot(start(1), start(2), '*');
hold on
plot( goal(1),  goal(2), '*');
draw_circle(p_o(:, 1), r_o);
hold on
draw_circle(p_o(:, 2), r_o);
hold on
plot([-0.2, -0.2], [-0.3, 1.2])
hold on
plot([0.8, 0.8], [-0.3, 1.2])
hold on
plot([-0.2, 0.8], [-0.3, -0.3])
hold on
plot([-0.2, 0.8], [1.2, 1.2])

xlim([-0.3, 0.9]);
ylim([-0.4, 1.3]);
xlabel('x [m]')
ylabel('y [m]')
axis equal;

figure
plot(t(1:num), hs(1:num))
xlabel('t')
ylabel('cbf h(s)');

figure
plot(t(1:num), Vs(1:num))
xlabel('t')
ylabel('clf h(s)');

end

function h = draw_circle(center,r)
hold on
th = 0:pi/50:2*pi;
xunit = r * cos(th) + center(1);
yunit = r * sin(th) + center(2);
h = plot(xunit, yunit);
hold off

end