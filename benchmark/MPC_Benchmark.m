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
                changing covairances from the true to measure adaptability
                (eps)
                try different initial conditions
                think of different noise profiles for sensors
                
                record MPC for mission characteristics
                root MSE

                
%}
rng(1);


%initial parameters
%traj = [-0.2;-0.2;0.2;0.001;0.001;0.001];
traj = [-10;-10;10;0.01;0.0001;0.0001];
%total_time = ARPOD_Benchmark.t_e; %equate the benchmark duration to eclipse time
total_time = 5000;
tstep = 5; % update every second
phase = ARPOD_Benchmark.calculatePhase(traj,0);

%MPC parameters
mpc_horizon = 50;
scale_mpcQ = 1;
scale_mpcR = 10;
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
stateEstimatorOption = 3;

%Setting up State Estimator Q and R matrices
%{

    For EKF, Q is the process covariance and R is sensor covariance.
    For PF, it's the same as EKF
    For MHE, Q is the cost weight matrix for state error. R is for meas
    error.

%}
if (stateEstimatorOption == 1)
    %EKF
    stateEstimator = ChaserEKF;
    stateEstimator = stateEstimator.initEKF(traj, 1e-10*eye(6)); %really trust initial estimate.

    % tunable parameters
    seQ = 1e-20*diag([1,1,1,1,1,1]);
    seR = diag([0.001,0.001,0.01]);
elseif (stateEstimatorOption == 2)
    %PF
    ess_threshold = 700;
    n_particles = 1000;

    stateEstimator = ChaserPF;
    stateEstimator = stateEstimator.initPF(traj.', 1e-50*eye(6), n_particles, ess_threshold);

    % tunable estimators
    seQ = 1e-20*diag([1,1,1,1,1,1]);
    seR = diag([0.001,0.001,0.01]);
else
    %moving horizon estimator
    stateEstimator = ChaserMHE;

    mhe_horizon = 20;
    forget_factor = 1;


    sense = ARPOD_Benchmark.sensor(traj, @() [0;0;0], phase);
    stateEstimator = stateEstimator.initMHE(traj,sense,mhe_horizon, forget_factor, tstep);


    seQ = 1e20*diag([1,1,1,1,1,1]); %arbitrarily large
    seR = diag([1e3, 1e3, 1e2]);
end

%setting up gaussian noise models
scale_sensorNoise = 1;
scale_dynamicNoise = 0.000000001;

%initialize statistics for graph
stats = ARPOD_Statistics;
stats = stats.initBenchmark(traj);

estTraj = traj;


for i = tstep:tstep:total_time
    disp(i)
    phase = ARPOD_Benchmark.calculatePhase(traj,false);
    %calculate the next "u" using MPC

    %vary the process noise depending based on benchmark
    %benchmark doesn't consider process noise :{
    %only considers sensor noise and varies it depending on phase
    if phase == 1
        noiseQ = @() [0;0;0;0;0;0];
        noiseR = @() mvnrnd([0;0;0], [0.001, 0.001, 0.01]).';
    elseif phase == 2
        noiseQ = @() [0;0;0;0;0;0];
        noiseR = @() 10*mvnrnd([0;0;0], [0.001, 0.001, 0.01]).';
    else
        noiseQ = @() [0;0;0;0;0;0];
        noiseR = @() mvnrnd([0;0;0], [0.001, 0.001, 1e-5]).';
    end


    if mpc_choice == 1 %linear
        u = ChaserMPC.controlMPC(traj,mpc_Q,mpc_R,mpc_horizon,tstep,ARPOD_Benchmark.m_c,phase);
    else %nonlinear 
        mpc = mpc.controlMPC(traj, mpc_Q, mpc_R, tstep, mpc_horizon, ARPOD_Benchmark.m_c, phase);
        u = mpc.warm_u(:,1);
    end

    % use the u to simulate next step of the chaser spacecraft
    traj = ARPOD_Benchmark.nextStep(traj.',u,tstep,noiseQ,1);
    % simulate sensor reading
    sense = ARPOD_Benchmark.sensor(traj,noiseR,phase);

    %given sensor reading read EKF
    stateEstimator = stateEstimator.estimate(u, sense, seQ, seR, tstep, phase);
    estTraj = stateEstimator.state;

    stats = stats.updateBenchmark(u, ARPOD_Benchmark.m_c, traj, estTraj, tstep, phase);

    if sqrt(traj(1).^2 + traj(2).^2 + traj(3).^2) < 0.001
        disp("Docked Early")
        break
    end
end

%graph results
stats.graphLinear(pi/3,pi/3);


