%testing the HCW equations

ts = linspace(0,100,101);
%width T = 1
T = 1;
mu_GM = 398600.4;
R = 1000;
n = sqrt(mu_GM / R.^3);
x0 = -100;
y0 = -100;
z0 = -100;
xdot0 = 10;
ydot0 = 20;
zdot0 = 15;
state0 = [x0;y0;z0;xdot0;ydot0;zdot0];

%verified these equations already.
fx = @(t) (4-3*cos(n*t))*x0 + sin(n*t)/n*xdot0 + 2/n*(1-cos(n*t))*ydot0;
fy = @(t) (6*(sin(n*t)-n*t)*x0 + y0 - 2/n*(1-cos(n*t))*xdot0 + (4*sin(n*t)-3*n*t)/n*ydot0);
fz = @(t) z0*cos(n*t) + zdot0/n*sin(n*t);

% This is the true value of the HCW Equations!
traj = @(t) [fx(t);fy(t);fz(t)];
trajs = zeros(3,length(ts));
trajsDiscrete = zeros(6,length(ts));

u = @(t) [0;0;0];
endt = 100;
[ts_,trajsNL] = nonlinearChaserDynamics.simulateMotion(state0,R,u,endt);

trajs(:,1) = state0(1:3,:);
trajsDiscrete(:,1) = state0;
for i = 2:length(ts)
    trajs(:,i) = traj( ts(i) );
    [pred, cov_] = ChaserEKF.prediction(trajsDiscrete(:,i-1), eye(6), [0;0;0], T, R, eye(6)); % use EKF prediction to sanity check if it works
    trajsDiscrete(:,i) = pred;
end

figure()
plot3(trajs(1,:), trajs(2,:), trajs(3,:),'-r'); % red is the true DiffEq solved HCW trajectory
hold on
plot3(trajsDiscrete(1,:), trajsDiscrete(2,:), trajsDiscrete(3,:),'-b'); % blue is the HCW trajectory code from the EKF (should give same results as red)
plot3(trajsNL(:,1), trajsNL(:,2), trajsNL(:,3),'-g'); % green is the nonlinear ode solved nonlinear trajectory (should look like red and blue but different)
hold off
title('Chaser Trajectory')
xlabel('x')
ylabel('y')
zlabel('z')
grid on

%figure()
%trajs(:,:) - trajsDiscrete(1:3,:)
%plot()