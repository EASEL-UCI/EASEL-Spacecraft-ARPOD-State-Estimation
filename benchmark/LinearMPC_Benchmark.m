%{
    Linear MPC Benchmark Script
    ----------------------------
        Description:
        ------------
            Given choice of Particle Filter, Extended Kalman Filter, and
            Moving Horizon Estimator, follow and estimate trajectory using
            Moving Horizon Estimator.

            Graph should give the estimated trajectory 
        
        TODO: Run EKF
        TODO: Run PF
        TODO: Run MHE

        Future features: 
        ----------------
           - Thruster Options
           - Discrete/Cont/Impulsive Choice
           - State estimation Option6
%}

%initial parameters
traj = [-100;-100;-100;0;0;0];
total_time = ARPOD_Benchmark.t_e; %equate the benchmark duration to eclipse time
tstep = 1; % update every second
phase = 1;
scale_sensorNoise = 0.1;
scale_dynamicNoise = 0.001;

%MPC parameters
mpc_horizon = 100;
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
stateEstimator = 1;

%Future features: 
%   Thruster Options
%   Discrete/Cont/Impulsive Choice
%   State estimation Option

%setting up gaussian noise models
noisePhase1 = @() transpose(scale_sensorNoise*mvnrnd([0;0;0], [1,1], 1));
noisePhase234 = @() transpose(scale_sensorNoise*mvnrnd([0;0;0], [1,1,0.001], 1));
noiseP = @() transpose(scale_dynamicNoise*mvnrnd([0;0;0;0;0;0], [1;1;1;0.1;0.1;0.1], 1));

%initialize statistics for graph
stats = ARPOD_Statistics;
stats.initBenchmark(traj);

sense = ARPOD_Benchmark.sensor(traj, @() [0;0], phase);


[xs,us] = ChaserMPC.optimizeLinear(traj,mpc_Q,mpc_R,mpc_horizon,tstep,1, 0.01);

%{
for i = 1+tstep:tstep:total_time
    phase = ARPOD_Benchmark.calculatePhase(traj,false);

    %calculate the next "u" using MPC

    traj = ARPOD_Benchmark.nextStep(traj,u,tstep,noiseP,phase);
    state = ARPOD_Benchmark.sensor(traj,noiseQ,phase);

    

    stats.updateBenchmark
end
%}

