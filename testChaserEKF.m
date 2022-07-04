%constants
mu_GM = 398600.4;
R = 100;
n = sqrt(mu_GM / R.^3);
x0 = -100;
y0 = -100;
z0 = -100;
xdot0 = 10;
ydot0 = 20;
zdot0 = 15;
state0 = [x0;y0;z0;xdot0;ydot0;zdot0];

%get the true values
%u = @(t) [1000*sin(20*t)+500*sin(500*cos(30*t));1000*sin(10*t)-500*cos(40*t);1000*sin(50*t)+500*cos(60*t)];
%u = @(t) [100*sin(t);1000*cos(t);20*sin(t)];
u = @(t) [t/1000;t*sin(t);t*cos(t)];
u0 = @(t) [0;0;0];
endt = 100;
[ts_,trajsOrbit] = nonlinearChaserDynamics.simulateMotion(state0,R,u0,endt); %get the true Trajectories
[ts,trajsTrue] = nonlinearChaserDynamics.simulateMotion(state0,R,u,endt); %get the true Trajectories

%System Noise
noise_mu = [0;0;0;0;0;0];
noise_cov = [1,1,1,1,1,1];
noise = @() mvnrnd(noise_mu,noise_cov,1); %white gaussian noise
%noise = @() [0,0,0,0,0,0];

%Sensor Noise
noise_mu = [0;0;0];
noise_cov = [1,1,1];
sensor_noise = @() transpose(mvnrnd(noise_mu,noise_cov,1)); %white gaussian noise
%sensor_noise = @() [0;0;0];

%noisify traj
noisy_traj = nonlinearChaserDynamics.noisifyMotion(trajsTrue, noise);

%noisify data
sense_data = ARPOD_Sensing.convertTrajs(noisy_traj); %get the sensor data equivalent of this
noisy_data = ARPOD_Sensing.noisifyData(sense_data,sensor_noise);

%get the estimation ready
[n_traj, dim_traj] = size(trajsTrue);
estimatedTrajCov = zeros(6,6,n_traj);
estimatedTrajCov(:,:,1) = zeros(6,6);

estimatedTraj = zeros(6,n_traj);
estimatedTraj(:,1) = state0;

for i = 2:length(ts)
    T = ts(i) - ts(i-1);
    [est_state, est_cov] = ChaserEKF.estimate(estimatedTraj(:,i-1), estimatedTrajCov(:,:,i-1), u( ts(i-1) ), T, R, sense_data(:,i), 1*eye(6), 1*eye(3));
    estimatedTraj(:,i) = est_state;
    estimatedTrajCov(:,:,i) = est_cov;
end

figure()
plot3(noisy_traj(:,1), noisy_traj(:,2), noisy_traj(:,3),'-r'); 
hold on
plot3(trajsOrbit(:,1), trajsOrbit(:,2), trajsOrbit(:,3),'-g');
%hold on
plot3(estimatedTraj(1,:), estimatedTraj(2,:), estimatedTraj(3,:),'-b');
hold off
title('Chaser Trajectory')
xlabel('x')
ylabel('y')
zlabel('z')
grid on
