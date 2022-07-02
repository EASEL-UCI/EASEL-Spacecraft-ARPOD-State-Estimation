traj0 = [-100;-100;-100;0;0;0]; %prev_traj
R = 10000;
u = @(t) [10*sin(t);10*sin(t);10*sin(t)];
T = 1;
[ts,trajs] = nonlinearChaserDynamics.simulateMotion(traj0,R,u,T);

noise_mu = [0;0;0;0;0;0];
noise_cov = [0.25,0.5,0.75,0.25,0.5,0.75];
noise = @() mvnrnd(noise_mu,noise_cov,1); %white gaussian noise
noisy_trajs = nonlinearChaserDynamics.noisifyMotion(trajs,noise); % noisify the trajectories produced

%getting sensor data from PHASE 1 and 2.
noise_mu = [0;0;0];
noise_cov = [0.25,0.5,0.75];
sensor_noise = @() transpose(mvnrnd(noise_mu,noise_cov,1)); %white gaussian noise
sense_data = ARPOD_Sensing.convertTrajs(noisy_trajs);
noisy_data = ARPOD_Sensing.noisifyData(sense_data, sensor_noise);

%Commented out because laptop can't handle this. :(
%{
figure()
plot3(trajs(:,1), trajs(:,2), trajs(:,3));
title('Chaser Trajectory')
xlabel('x')
ylabel('y')
zlabel('z')
grid on
%}