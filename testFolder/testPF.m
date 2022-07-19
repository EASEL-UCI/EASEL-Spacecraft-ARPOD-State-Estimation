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
u = @(t) [t/1000;t*sin(t);t*cos(t)];
endt = 50;
T = 1;
N = endt/T;
[ts,trajsTrue] = nonlinearChaserDynamics.simulateMotion(state0,R,u,endt,T); %get the true Trajectories
sense_data = ARPOD_Sensing.convertTrajs(trajsTrue); %get the sensor data equivalent of this

noise_mu = [0;0;0];
noise_cov = [0.25,0.5,0.75];
noise = @() transpose(mvnrnd(noise_mu,noise_cov,1)); %white gaussian noise
noisy_data = ARPOD_Sensing.noisifyData(sense_data, noise);
[n_traj, dim_traj] = size(trajsTrue);

estimatedTraj = zeros(6,n_traj);
estimatedTraj(:,1) = state0;
[weights, particles] = ChaserPF.initialize_particles(state0, [0,0,0,0,0,0], 1000);
for i = 2:length(ts)
    disp(i)
    us = @(t) u(t+ts(i-1));
    [weights, particles] = ChaserPF.nonlinearPF(weights, particles, noisy_data(:,i), 500, eye(6), eye(3), us, R, T);
    traj = ChaserPF.estimateState(weights, particles);
    estimatedTraj(:,i) = traj;
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