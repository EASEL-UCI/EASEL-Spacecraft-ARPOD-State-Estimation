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

%define the continuous control input
u = @(t) [t/1000;t*sin(t);t*cos(t)];
u0 = @(t) [0;0;0];
endt = 25;
tstep = 0.5;
N = endt / tstep;
[ts_,trajsOrbit] = nonlinearChaserDynamics.simulateMotion(state0,R,u0,endt,tstep); %get the true Trajectories
[ts,trajsTrue] = nonlinearChaserDynamics.simulateMotion(state0,R,u,endt,tstep); %get the true Trajectories

%System Noise
noise_mu = [0;0;0;0;0;0];
noise_cov = 10*[1,1,1,1,1,1];
noise = @() mvnrnd(noise_mu,noise_cov,1); %white gaussian noise
%noise = @() [0,0,0,0,0,0];

%Sensor Noise
noise_mu = [0;0;0];
noise_cov = 5*[1,1,1];
sensor_noise = @() transpose(mvnrnd(noise_mu,noise_cov,1)); %white gaussian noise
%sensor_noise = @() [0;0;0];
%noisify traj
noisy_traj = nonlinearChaserDynamics.noisifyMotion(trajsTrue, noise);

%noisify data
sense_data = ARPOD_Sensing.convertTrajs(noisy_traj); %get the sensor data equivalent of this
noisy_data = ARPOD_Sensing.noisifyData(sense_data,sensor_noise);

%get initial guess
n_horizon = 10;
states_horizon = zeros(6,n_horizon);
states_horizon(:,1) = state0;

%get weight matrices for optimization
weightW = zeros(6,6,N);
weightW(:,:,1) = eye(6);
weightV = zeros(3,3,N);
weightV(:,:,1) = eye(3);
us = zeros(3,N);

for i = 1:N
    us(:,i) = u( ts(i) );
end

for i = 2:n_horizon
    states_horizon(:,i) = ChaserMHE.linearDynamics(states_horizon(:,i-1), us(:,i-1), R, tstep);
    weightW(:,:,i) = 1*eye(6);
    weightV(:,:,i) = 1*eye(3);
end

%weights for optimization
options = optimoptions(@fmincon, 'Algorithm', 'sqp', 'MaxIterations', 2000, 'ConstraintTolerance', 1e-6);
states = ChaserMHE.optimize(noisy_data(:,1:10), us(:,1:10), states_horizon, states_horizon(:,1), weightW, weightV, n_horizon, tstep, R, options);

%simulation loop
trajsEstimated = zeros(6,N);
trajsEstimated(:,1:n_horizon) = states;

disp( trajsTrue(1:10,:).' )
disp( trajsEstimated(:,1:10) )
for i = 2:N-n_horizon+1
    states = zeros(6,n_horizon);
    states(:,1:n_horizon-1) = trajsEstimated(:,i:n_horizon+i-2); %warm-starting
    states(:,n_horizon) = ChaserMHE.linearDynamics( states(:,n_horizon-1), us(:,n_horizon+i-2 ), R, tstep);
    %warm starting using previous MHE solution

    states = ChaserMHE.optimize(noisy_data(:,i:n_horizon+i-1), us(:,i:n_horizon+i-1), states, states(:,1), weightW, weightV, n_horizon, tstep, R, options);
    trajsEstimated(:,i:n_horizon+i-1) = states;
end

figure()
plot3(noisy_traj(:,1), noisy_traj(:,2), noisy_traj(:,3),'-r'); 
hold on
plot3(trajsOrbit(:,1), trajsOrbit(:,2), trajsOrbit(:,3),'-g');
%hold on
plot3(trajsEstimated(1,:), trajsEstimated(2,:), trajsEstimated(3,:),'-b');
hold off
title('Chaser Trajectory')
xlabel('x')
ylabel('y')
zlabel('z')
grid on