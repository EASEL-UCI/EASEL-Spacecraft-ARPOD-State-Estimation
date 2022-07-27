%constants
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

%get the true values
u = @(t) [0;0;0]; 
endt = 100;
[ts,trajsTrue] = nonlinearChaserDynamics.simulateMotion(state0,R,u,endt,1); %get the true Trajectories
sense_data = ARPOD_Sensing.convertTrajs(trajsTrue); %get the sensor data equivalent of this

[n_traj, dim_traj] = size(trajsTrue);
estimatedTrajCov = zeros(6,6,n_traj);
estimatedTrajCov(:,:,1) = eye(6,6);

estimatedTraj = zeros(6,n_traj);
estimatedTraj(:,1) = state0;
for i = 2:length(ts)
    T = ts(i) - ts(i-1);
    [est_state, est_cov] = ChaserEKF.estimate(estimatedTraj(:,i-1), estimatedTrajCov(:,:,i-1), [0;0;0], T, R, sense_data(:,i), 1*eye(6), 0.001*eye(3), 2);
    estimatedTraj(:,i) = est_state;
    estimatedTrajCov(:,:,i) = est_cov;
end

figure()
plot3(trajsTrue(:,1), trajsTrue(:,2), trajsTrue(:,3),'-r'); 
hold on
plot3(estimatedTraj(1,:), estimatedTraj(2,:), estimatedTraj(3,:),'-b');
hold off
title('Chaser Trajectory')
xlabel('x')
ylabel('y')
zlabel('z')
grid on

