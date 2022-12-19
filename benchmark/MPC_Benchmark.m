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
%traj = [-1;-0;0;0.01;0.01;0.001];
%traj = [-5;-5;5;0.01;0.0001;0.0001];
%traj = [-10;10;10;-0.01;0.001;0.001];
traj = [-3.37859,-3.288,0.4861,0.00024719,-0.00091536,0.00092736].';
%traj = [3.1472,4.0579,-3.7301,0.00082675,0.00026472,-0.00080492].';


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
stateEstimatorOption = 4;

%Setting up State Estimator Q and R matrices
%{


    For EKF, Q is the process covariance and R is sensor covariance.
    For PF, it's the same as EKF
    For MHE, Q is the cost weight matrix for state error. R is for meas
    error.

%}

%1e-5
process_noise = 0*[1,1,1,1e-20,1e-20,1e-20];
if (stateEstimatorOption == 1)
    %EKF
    stateEstimator = ChaserEKF;
    stateEstimator = stateEstimator.initEKF(traj, 1e-50*eye(6)); %really trust initial estimate.

% tunable parameters
    if sum(process_noise) == 0
        seQ = 1e-20*diag([1,1,1,1,1,1]);
        seR = diag([0.001,0.001,0.001]);
    else
        seQ = diag(zeros(1,6)+process_noise);
        seR = diag([0.001,0.001,0.001]);
    end
elseif (stateEstimatorOption == 2)
    %PF
    ess_threshold = 700;
    n_particles = 1000;

    stateEstimator = ChaserPF;
    stateEstimator = stateEstimator.initPF(traj.', 1e-50*eye(6), n_particles, ess_threshold);

    % tunable estimators
    seQ = diag(zeros(1,6)+process_noise);
    seR = 1e5*diag([0.001,0.001,0.01]);
elseif stateEstimatorOption == 3
    %moving horizon estimator
    stateEstimator = ChaserMHE;

    mhe_horizon = 10;
    forget_factor = 1;  


    sense = ARPOD_Benchmark.sensor(traj, @() [0;0;0], phase);
    stateEstimator = stateEstimator.initMHE(traj,sense,mhe_horizon, forget_factor, tstep);


    if process_noise == 0
        seQ = 1e20*diag([1,1,1,1,1,1]); %arbitrarily large
    else
        seQ = max(process_noise(:)).^(-1)*1e3*diag([1,1,1,1,1,1]); %arbitrarily large
    end
    seR = diag([1e3, 1e3, 1e2]);

elseif stateEstimatorOption == 4 %MHE unconstr
    stateEstimator = ChaserMHE_Unconstr;
    mhe_horizon = 10;


    if process_noise == 0
        seQ = 1e20*diag([1,1,1,1,1,1]); %arbitrarily large
    else
        seQ = max(process_noise(:)).^(-1)*diag([1,1,1,1,1,1]); %arbitrarily large
    end
    seR = diag([1e3, 1e3, 1e2]);

    sense = ARPOD_Benchmark.sensor(traj, @() [0;0;0], phase);
    stateEstimator = stateEstimator.init(traj, sense, mhe_horizon, tstep);

else %MHE EKF
    stateEstimator = ChaserMHE_EKF;
    mhe_horizon = 10;
    forget_factor = 1;

    if process_noise == 0
        seQ = 1e20*diag([1,1,1,1,1,1]); %arbitrarily large
        ekfQ = 1e-20*diag([1,1,1,1,1,1]);
        ekfR = diag([0.001,0.001,0.01]);
    else
        seQ = max(process_noise(:)).^(-1)*diag([1,1,1,1,1,1]); %arbitrarily large
        ekfQ = diag(zeros(1,6)+process_noise);
        ekfR = diag([0.001,0.001,0.01]);
    end
    seR = 1e3*diag([1,1,1,1,1,1]);

    stateEstimator = stateEstimator.init(traj, mhe_horizon, forget_factor, tstep, ekfQ, ekfR);
end

%initialize EKF for MHJE
if stateEstimatorOption >= 3 && stateEstimatorOption < 5
    ekf = ChaserEKF;
    ekf = ekf.initEKF(traj, 1e-10*eye(6)); %really trust initial estimate.

    % tunable parameters
    if process_noise == 0
        ekfQ = 1e-20*diag([1,1,1,1,1,1]);
        ekfR = diag([0.001,0.001,0.01]);
    else
        ekfQ = diag(zeros(1,6)+process_noise);
        ekfR = diag([0.001,0.001,0.01]);
    end
end

%initialize statistics for graph
stats = ARPOD_Statistics;
stats = stats.initBenchmark(traj);

estTraj = traj;


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
    tic
    if stateEstimatorOption >= 3 && stateEstimatorOption < 5 && i <= mhe_horizon*(tstep+1)
        ekf = ekf.estimate(u,sense,ekfQ,ekfR,tstep,phase);
        stateEstimator = stateEstimator.estimate(u, sense, seQ, seR, tstep, phase);
        estTraj = ekf.state;
    else
        stateEstimator = stateEstimator.estimate(u, sense, seQ, seR, tstep, phase);
        estTraj = stateEstimator.state;
    end
    estTime = toc;

    stats = stats.updateBenchmark(u, ARPOD_Benchmark.m_c, traj, estTraj, tstep, estTime, phase);

    if sqrt(estTraj(1).^2 + estTraj(2).^2 + estTraj(3).^2) < 1e-3
        disp("Docked Early")
        break
    end
end

%graph results
theta = 60;
stats.graphLinear(theta * pi / 180,theta * pi / 180);

