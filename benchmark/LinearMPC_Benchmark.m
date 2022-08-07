%{
    Linear MPC Benchmark Script
    ----------------------------
        Description:
        ------------
            Given choice of Particle Filter, Extended Kalman Filter, and
            Moving Horizon Estimator, follow and estimate trajectory using
            Moving Horizon Estimator.

            Graph should give the estimated trajectory 
        
        TODO: Run PF
        TODO: Run MHE

        Future features: 
        ----------------
           - Thruster Options
           - Discrete/Cont/Impulsive Choice
           - State estimation Option6
%}

%initial parameters
traj = [-0.5;-0.5;-0.5;0;0;0];
%total_time = ARPOD_Benchmark.t_e; %equate the benchmark duration to eclipse time
total_time = 1000;
tstep = 1; % update every second
phase = ARPOD_Benchmark.calculatePhase(traj,0);

%MPC parameters
mpc_horizon = 25;
scale_mpcQ = 100;
scale_mpcR = 1;
mpc_Q = scale_mpcQ*[10,0,0,0,0,0;
        0,10,0,0,0,0;
        0,0,10,0,0,0;
        0,0,0,1,0,0;
        0,0,0,0,1,0;
        0,0,0,0,0,1];
mpc_R = scale_mpcR*eye(3);

%{
    State estimator choice:
    -----------------------
        1: Extended Kalman Filter
        2: Particle Filter
        3: Moving Horizon Estimator
%}
stateEstimatorOption = 1;

%Setting up State Estimator Q and R matrices
%{
    For EKF, Q is the process covariance and R is sensor covariance.
    For PF, it's the same as EKF
    For MHE, Q is the cost weight matrix for state error. R is for meas
    error.
%}
if (stateEstimatorOption == 1)
    stateEstimator = ChaserEKF;
    stateEstimator = stateEstimator.initEKF(traj, 0.00001*eye(6)); %really trust initial estimate.

    scale_Q = 0.001;
    scale_R = 0.01;
    seQ = scale_Q*eye(6);
    seR = scale_R*eye(3);
end

%setting up gaussian noise models
scale_sensorNoise = 0.1;
scale_dynamicNoise = 0;
noiseQ = @() transpose(scale_sensorNoise*mvnrnd([0;0;0], [1,1,0.001], 1));
noiseP = @() transpose(scale_dynamicNoise*mvnrnd([0;0;0;0;0;0], [1,1,1,0.1,0.1,0.1], 1));

%initialize statistics for graph
stats = ARPOD_Statistics;
stats = stats.initBenchmark(traj);

sense = ARPOD_Benchmark.sensor(traj, @() [0;0;0], phase);
estTraj = traj;


for i = 1+tstep:tstep:total_time
    disp(i)
    phase = ARPOD_Benchmark.calculatePhase(traj,false);
    %calculate the next "u" using MPC
    u = ChaserMPC.controlMPC(traj,mpc_Q,mpc_R,mpc_horizon,tstep,ARPOD_Benchmark.m_c,phase,1);

    % use the u to simulate next step of the chaser spacecraft
    traj = ARPOD_Benchmark.nextStep(traj.',u,tstep,noiseP,1);
    % simulate sensor reading
    sense = ARPOD_Benchmark.sensor(traj,noiseQ,phase);


    %given sensor reading read EKF
    stateEstimator = stateEstimator.estimate(u,sense, seQ, seR, tstep, phase);
    estTraj = stateEstimator.state;

    stats = stats.updateBenchmark(u, ARPOD_Benchmark.m_c, traj, estTraj, tstep, phase);
end
stats.graphLinear(pi/3,pi/3);


