%{
    Linear MPC Benchmark Script
    ----------------------------
        Description:
        ------------
            Given choice of Particle Filter, Extended Kalman Filter, and
            Moving Horizon Estimator, follow and estimate trajectory using
            Moving Horizon Estimator.

            Graph should give the estimated trajectory 

        TODO: Run Linear MPC from the ChaserMPC
        TODO: Run MHE from ChaserMHE
%}

%initial parameters
traj = [-100;-100;-100;100;100;100];
total_time = ARPOD_Benchmark.t_e; %equate the benchmark duration to eclipse time
mpc_horizon = 25; % set linear mpc horizon
mhe_horizon = 25; % set nonlinear mhe horizon
tstep = 1; % update every second
phase = 1;
scale_sensorNoise = 0.1;
scale_dynamicNoise = 0.001;

%Future features: 
%

%setting up gaussian noise models
noiseQ = @() transpose(scale_sensorNoise*mvnrnd([0;0;0], [1,1,0.001], 1));
noiseP = @() transpose(scale_dynamicNoise*mvnrnd([0;0;0;0;0;0], [1;1;1;0.1;0.1;0.1], 1));

%setting up variables for mhe
mhe_stateWindow = [];
mhe_sensorWindow = [];

%initialize statistics for graph
stats = ARPOD_Statistics;
stats.initBenchmark(traj);

sense = ARPOD_Benchmark.sensor(traj, noiseQ, phase);
mhe_stateWindow = [traj];
mhe_sensorWindow = [sense];

for i = 1+tstep:tstep:total_time
    phase = ARPOD_Benchmark.calculatePhase(traj,false);

    %calculate the next "u" using MPC

    traj = ARPOD_Benchmark.nextStep(traj,u,tstep,noiseP,phase);


    if i < mhe_horizon
        %estimate over all of previous timesteps

    else
        %estimate over only last mhe_horizon timesteps

    end
    stats.updateBenchmark
end


