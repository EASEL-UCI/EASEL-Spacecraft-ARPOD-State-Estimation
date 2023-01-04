%{
    MPC Benchmark Script
    --------------------
        Description:
        ------------
            Given choice of Particle Filter, Extended Kalman Filter, and
            Moving Horizon Estimator, follow and estimate trajectory using
            Moving Horizon Estimator.

            Graph should give the estimated trajectory 


        Tests: different phases
                init MHE    
                write a script to comapre all estimators in a side-by-side
                monte carlo trajectories 
                write stuff into a file
                plot the stuff in file for side-by-side comparison
                use file names + values for automated mc


                packet dropouts
                packet delay
%}
close all
clear
clc
rng(1);


%initial parameters
%traj = [-1;-2;-1;0.0001;0.0001;0.001];
%traj = [-6;-6;6;0.01;0.0001;0.0001];
%traj = [-10;10;10;-0.01;0.001;0.001];
%traj = [0.37859,-3.288,2.4861,0.00024719,-0.00091536,0.00092736].';
%traj = [3.1472,4.0579,-3.7301,0.00082675,0.00026472,-0.00080492].';
traj = [-3.675, -3.8016,  4.4314, 0.0003, 0.0006, 0.0003].';

%total_time = ARPOD_Benchmark.t_e; %equate the benchmark duration to eclipse time
total_time = 1500;
tstep = 1; % update every 0.33 seconds
phase = ARPOD_Benchmark.calculatePhase(traj,0);

%MPC parameters
mpc_horizon = 10;
scale_mpcQ = 1;
scale_mpcR = 100;
mpc_Q = scale_mpcQ*[1,0,0,0,0,0;
        0,1,0,0,0,0;
        0,0,1,0,0,0;
        0,0,0,100,0,0;
        0,0,0,0,100,0;
        0,0,0,0,0,100];
mpc_R = scale_mpcR*eye(3);

%{
    Model Predictive Control choice:
    --------------------------------
        1. Linear MPC --> quadprog
        2. Nonlinear MPC --> fmincon
%}
mpc_choice = 1;
if mpc_choice == 2
    mpc = ChaserNLMPC;
end

%{
    State estimator choice:
    -----------------------
        1: Extended Kalman Filter
        2: Particle Filter
        3: Moving Horizon Estimator
%}
stateEstimatorOption = 2;

%Setting up State Estimator Q and R matrices
%{


    For EKF, Q is the process covariance and R is sensor covariance.
    For PF, it's the same as EKF
    For MHE, Q is the cost weight matrix for state error. R is for meas
    error.

%}

%1e-5
process_noise = 1e-5*[1,1,1,1e-20,1e-20,1e-20];

stateEstimator1 = ChaserEKF;
stateEstimator1 = stateEstimator1.initEKF(traj, 1e-50*eye(6)); %really trust initial estimate.

% tunable parameters
if sum(process_noise) == 0
    seQ1 = 1e-20*diag([1,1,1,1,1,1]);
    seR1 = diag([0.001,0.001,0.001]);
else
    seQ1 = diag(zeros(1,6)+process_noise);
    seR1 = diag([0.001,0.001,0.001]);
end


%PF
ess_threshold = 700;
n_particles = 1000;

stateEstimator2 = ChaserPF;
stateEstimator2 = stateEstimator2.initPF(traj.', 1e-50*eye(6), n_particles, ess_threshold);

% tunable estimators
seQ2 = diag(zeros(1,6)+process_noise);
seR2 = 1e3*diag([0.001,0.001,0.01]);


trackEKFTrueTraj = [traj];
trackEKFTraj = [];

trackPFTrueTraj = [traj];
trackPFTraj = [];


estTraj = traj;

seQ = seQ1;
seR = seR1;

traj0 = traj;
stateEstimator = stateEstimator1;

%EKF
for i = tstep:tstep:total_time
    disp(i)
    phase = ARPOD_Benchmark.calculatePhase(estTraj,false);
    %calculate the next "u" using MPC

    %vary the process noise depending based on benchmark
    %benchmark doesn't consider process noise :{
    %only considers sensor noise and varies it depending on phase
    process_noise_noise = 0.000;
    noise_noise = 0;
    if phase == 1
        noiseQ = @() mvnrnd([0;0;0;0;0;0], [0,0,0,0,0,0] + process_noise).';
        %noiseQ = @() mvnrnd([0;0;0;0;0;0], [0,0,0,0,0,0]).';
        noiseR = @() mvnrnd([0;0;0], [0.001, 0.001, 0.01] + noise_noise).';
    elseif phase == 2
        noiseQ = @() mvnrnd([0;0;0;0;0;0], [0,0,0,0,0,0] + (process_noise + process_noise_noise)*0.001).';
        noiseR = @() mvnrnd([0;0;0], [0.001, 0.001, 0.01] + noise_noise).';
    else
        noiseQ = @() mvnrnd([0;0;0;0;0;0], [0,0,0,0,0,0] + (process_noise + process_noise_noise)*0.0001).';
        noiseR = @() mvnrnd([0;0;0], [0.001, 0.001, 1e-5] + noise_noise).';
    end


    if mpc_choice == 1 %linear
        if (i == tstep)
            x0 = -1;
            [x0,u] = ChaserMPC.controlMPC(estTraj,x0,mpc_Q,mpc_R,mpc_horizon,tstep,ARPOD_Benchmark.m_c,phase);
        else
            [A,B] = ARPOD_Benchmark.linearDynamics(tstep);
            x0 = [ x0(10:end,:); [0;0;0]; A*x0(end-5:end)];
            [x0,u] = ChaserMPC.controlMPC(estTraj,x0,mpc_Q,mpc_R,mpc_horizon,tstep,ARPOD_Benchmark.m_c,phase);
        end
    else %nonlinear 
        mpc = mpc.controlMPC(estTraj, mpc_Q, mpc_R, tstep, mpc_horizon, ARPOD_Benchmark.m_c, phase);
        u = mpc.warm_u(:,1);
    end

    % use the u to simulate next step of the chaser spacecraft
    traj = ARPOD_Benchmark.nextStep(traj.',u,tstep,noiseQ,1);

    % simulate sensor reading
    sense = ARPOD_Benchmark.sensor(traj,noiseR,phase);

    %given sensor reading read EKF
    stateEstimator = stateEstimator.estimate(u, sense, seQ, seR, tstep, phase);
    estTraj = stateEstimator.state;

    trackEKFTrueTraj = [trackEKFTrueTraj, traj];
    trackEKFTraj = [trackEKFTraj, estTraj];

    if sqrt(estTraj(1).^2 + estTraj(2).^2 + estTraj(3).^2) < 1e-3
        disp("Docked Early")
        break
    end
end

traj = traj0;
estTraj = traj;


seQ = seQ2;
seR = seR2;
stateEstimator = stateEstimator2;

for i = tstep:tstep:total_time
    disp(i)
    phase = ARPOD_Benchmark.calculatePhase(estTraj,false);
    %calculate the next "u" using MPC

    %vary the process noise depending based on benchmark
    %benchmark doesn't consider process noise :{
    %only considers sensor noise and varies it depending on phase
    process_noise_noise = 0.000;
    noise_noise = 0;
    if phase == 1
        noiseQ = @() mvnrnd([0;0;0;0;0;0], [0,0,0,0,0,0] + process_noise).';
        noiseR = @() mvnrnd([0;0;0], [0.001, 0.001, 0.01] + noise_noise).';
    elseif phase == 2
        noiseQ = @() mvnrnd([0;0;0;0;0;0], [0,0,0,0,0,0] + (process_noise + process_noise_noise)*0.1).';
        noiseR = @() mvnrnd([0;0;0], [0.001, 0.001, 0.01] + noise_noise).';
    else
        noiseQ = @() mvnrnd([0;0;0;0;0;0], [0,0,0,0,0,0] + (process_noise + process_noise_noise)*0.01).';
        noiseR = @() mvnrnd([0;0;0], [0.001, 0.001, 1e-5] + noise_noise).';
    end


    if mpc_choice == 1 %linear
        if (i == tstep)
            x0 = -1;
            [x0,u] = ChaserMPC.controlMPC(estTraj,x0,mpc_Q,mpc_R,mpc_horizon,tstep,ARPOD_Benchmark.m_c,phase);
        else
            [A,B] = ARPOD_Benchmark.linearDynamics(tstep);
            x0 = [ x0(10:end,:); [0;0;0]; A*x0(end-5:end)];
            [x0,u] = ChaserMPC.controlMPC(estTraj,x0,mpc_Q,mpc_R,mpc_horizon,tstep,ARPOD_Benchmark.m_c,phase);
        end
    else %nonlinear 
        mpc = mpc.controlMPC(estTraj, mpc_Q, mpc_R, tstep, mpc_horizon, ARPOD_Benchmark.m_c, phase);
        u = mpc.warm_u(:,1);
    end

    % use the u to simulate next step of the chaser spacecraft
    traj = ARPOD_Benchmark.nextStep(traj.',u,tstep,noiseQ,1);

    % simulate sensor reading
    sense = ARPOD_Benchmark.sensor(traj,noiseR,phase);

    %given sensor reading read EKF
    stateEstimator = stateEstimator.estimate(u, sense, seQ, seR, tstep, phase);
    estTraj = stateEstimator.state;
    
    trackPFTrueTraj = [trackPFTrueTraj, traj];
    trackPFTraj = [trackPFTraj, estTraj];

    if sqrt(estTraj(1).^2 + estTraj(2).^2 + estTraj(3).^2) < 1e-3
        disp("Docked Early")
        break
    end
end

theta1 = 60 * pi /180;
theta2 = theta1;

set(groot,'defaultAxesTickLabelInterpreter','latex');  
set(groot,'defaulttextinterpreter','latex');
set(groot,'defaultLegendInterpreter','latex');

fsize = 20; % figure fontsize
lw = 2; % linewidth

figure(1)
subplot(1,2,1);
%draw phase 2 sphere boundary
[x,y,z] = sphere(10);
r = ARPOD_Benchmark.rho_r;
scatter3(r*x(:),r*y(:),r*z(:), 3,'o', 'filled', 'MarkerEdgeColor', 'c');
axis equal
hold on

%draw phase 3 sphere boundary
[x,y,z] = sphere(5);
r = ARPOD_Benchmark.rho_d;
scatter3(r*x(:),r*y(:),r*z(:), 3, 'o', 'filled', 'MarkerEdgeColor', 'm');

c = ARPOD_Benchmark.rho_d;
%drawing pillars of pyramid
%plot3([0,-sin(theta2/2)*c], [0,-sin(theta1/2)*c], [0,-cos(theta1/2)*c], 'g');
plot3([0,-cos(theta1/2)*c], [0,-sin(theta2/2)*c], [0,-sin(theta1/2)*c], 'g');
%plot3([0,-sin(theta2/2)*c], [0,sin(theta1/2)*c], [0,-cos(theta1/2)*c], 'g');
plot3([0,-cos(theta1/2)*c], [0,sin(theta1/2)*c], [0,-sin(theta2/2)*c], 'g');

%plot3([0,sin(theta2/2)*c], [0,-sin(theta1/2)*c], [0,-cos(theta1/2)*c], 'g');
plot3([0,-cos(theta1/2)*c], [0,-sin(theta1/2)*c],[0,sin(theta2/2)*c],  'g');

%plot3([0,sin(theta2/2)*c], [0,sin(theta1/2)*c], [0,-cos(theta1/2)*c], 'g');
plot3([0,-cos(theta1/2)*c], [0,sin(theta1/2)*c], [0,sin(theta2/2)*c],  'g');

%drawing base of pyramid
%plot3([-sin(theta2/2)*c, sin(theta2/2)*c], [-sin(theta1/2)*c,-sin(theta1/2)*c], [-cos(theta1/2)*c,-cos(theta1/2)*c], 'g');
plot3([-cos(theta1/2)*c,-cos(theta1/2)*c], [-sin(theta1/2)*c,-sin(theta1/2)*c],[-sin(theta2/2)*c, sin(theta2/2)*c], 'g');

%plot3([-sin(theta2/2)*c, sin(theta2/2)*c], [sin(theta1/2)*c, sin(theta1/2)*c], [-cos(theta1/2)*c,-cos(theta1/2)*c], 'g');
plot3([-cos(theta1/2)*c,-cos(theta1/2)*c],[sin(theta1/2)*c, sin(theta1/2)*c], [-sin(theta2/2)*c, sin(theta2/2)*c],'g');

%plot3([-sin(theta2/2)*c, -sin(theta2/2)*c], [-sin(theta1/2)*c,sin(theta1/2)*c], [-cos(theta1/2)*c,-cos(theta1/2)*c], 'g');
plot3([-cos(theta1/2)*c,-cos(theta1/2)*c], [-sin(theta1/2)*c,sin(theta1/2)*c], [-sin(theta2/2)*c, -sin(theta2/2)*c], 'g');

%plot3([sin(theta2/2)*c, sin(theta2/2)*c], [-sin(theta1/2)*c, sin(theta1/2)*c], [-cos(theta1/2)*c,-cos(theta1/2)*c], 'g');
plot3([-cos(theta1/2)*c,-cos(theta1/2)*c],[-sin(theta1/2)*c, sin(theta1/2)*c], [sin(theta2/2)*c, sin(theta2/2)*c], 'g');

%draw truechaserTraj
plot3(trackEKFTrueTraj(1,:), trackEKFTrueTraj(2,:), trackEKFTrueTraj(3,:), 'r');
%draw estchaserTraj
plot3(trackEKFTraj(1,:), trackEKFTraj(2,:), trackEKFTraj(3,:), 'b');
hold off

%draw target position
%draw chaserStart
%draw chaserEnd
%draw 
title("Extended Kalman Filter Trajectory")
xlabel("x [km]")
ylabel("y [km]")
zlabel("z [km]")
xlim([-1.5,1.5])
ylim([-1.5,1.5])
zlim([-1.5,1.5])
set(gca,'fontsize',fsize)
set(gcf,'Position', [10 10 500 500])
set(gca, 'TickLabelInterpreter', 'latex')


subplot(1,2,2);
set(groot,'defaultAxesTickLabelInterpreter','latex');  
set(groot,'defaulttextinterpreter','latex');
set(groot,'defaultLegendInterpreter','latex');

fsize = 20; % figure fontsize
lw = 2; % linewidth

[x,y,z] = sphere(10);
r = ARPOD_Benchmark.rho_r;
scatter3(r*x(:),r*y(:),r*z(:), 5,'o', 'filled', 'MarkerEdgeColor', 'c');
axis equal
hold on

[x,y,z] = sphere(5);
r = ARPOD_Benchmark.rho_d;
scatter3(r*x(:),r*y(:),r*z(:), 5, 'o', 'filled', 'MarkerEdgeColor', 'm');
c = ARPOD_Benchmark.rho_d;
%drawing pillars of pyramid
%plot3([0,-sin(theta2/2)*c], [0,-sin(theta1/2)*c], [0,-cos(theta1/2)*c], 'g');
plot3([0,-cos(theta1/2)*c], [0,-sin(theta2/2)*c], [0,-sin(theta1/2)*c], 'g');
%plot3([0,-sin(theta2/2)*c], [0,sin(theta1/2)*c], [0,-cos(theta1/2)*c], 'g');
plot3([0,-cos(theta1/2)*c], [0,sin(theta1/2)*c], [0,-sin(theta2/2)*c], 'g');

%plot3([0,sin(theta2/2)*c], [0,-sin(theta1/2)*c], [0,-cos(theta1/2)*c], 'g');
plot3([0,-cos(theta1/2)*c], [0,-sin(theta1/2)*c],[0,sin(theta2/2)*c],  'g');

%plot3([0,sin(theta2/2)*c], [0,sin(theta1/2)*c], [0,-cos(theta1/2)*c], 'g');
plot3([0,-cos(theta1/2)*c], [0,sin(theta1/2)*c], [0,sin(theta2/2)*c],  'g');

%drawing base of pyramid
%plot3([-sin(theta2/2)*c, sin(theta2/2)*c], [-sin(theta1/2)*c,-sin(theta1/2)*c], [-cos(theta1/2)*c,-cos(theta1/2)*c], 'g');
plot3([-cos(theta1/2)*c,-cos(theta1/2)*c], [-sin(theta1/2)*c,-sin(theta1/2)*c],[-sin(theta2/2)*c, sin(theta2/2)*c], 'g');

%plot3([-sin(theta2/2)*c, sin(theta2/2)*c], [sin(theta1/2)*c, sin(theta1/2)*c], [-cos(theta1/2)*c,-cos(theta1/2)*c], 'g');
plot3([-cos(theta1/2)*c,-cos(theta1/2)*c],[sin(theta1/2)*c, sin(theta1/2)*c], [-sin(theta2/2)*c, sin(theta2/2)*c],'g');

%plot3([-sin(theta2/2)*c, -sin(theta2/2)*c], [-sin(theta1/2)*c,sin(theta1/2)*c], [-cos(theta1/2)*c,-cos(theta1/2)*c], 'g');
plot3([-cos(theta1/2)*c,-cos(theta1/2)*c], [-sin(theta1/2)*c,sin(theta1/2)*c], [-sin(theta2/2)*c, -sin(theta2/2)*c], 'g');

%plot3([sin(theta2/2)*c, sin(theta2/2)*c], [-sin(theta1/2)*c, sin(theta1/2)*c], [-cos(theta1/2)*c,-cos(theta1/2)*c], 'g');
plot3([-cos(theta1/2)*c,-cos(theta1/2)*c],[-sin(theta1/2)*c, sin(theta1/2)*c], [sin(theta2/2)*c, sin(theta2/2)*c], 'g');

plot3(trackPFTrueTraj(1,:), trackPFTrueTraj(2,:), trackPFTrueTraj(3,:), 'r');
plot3(trackPFTraj(1,:), trackPFTraj(2,:), trackPFTraj(3,:), 'b');

title("Particle Filter Trajectory")
xlabel("x [km]")
ylabel("y [km]")
zlabel("z [km]")
xlim([-1.5,1.5])
ylim([-1.5,1.5])
zlim([-1.5,1.5])
set(gca,'fontsize',fsize)
set(gcf,'Position', [10 10 500 500])
set(gca, 'TickLabelInterpreter', 'latex')
hold off

