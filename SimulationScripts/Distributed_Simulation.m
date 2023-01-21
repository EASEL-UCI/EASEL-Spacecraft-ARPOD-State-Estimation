% Distributed_Simulation.m
% -------------------------
%{
    High Performance (Computing) Script that makes the most out of CPU
    to run simulations. (save up on time)

    We will avoid using GPU's in order to make this agnostic

    Performance Optimization:
        - Simulation can be split into "m" independent routines.
        - Since they are independent, we can divide the work into "m/n" equal parts for "n" workers.

%}

%how many workers to run to split the simulation routine
delete(gcp('nocreate'))

n_workers = 8;
mc_length = 50;


start_phase = 1;

% 1 = EKF
% 2 = PF
% 3 = constrained MHE
% 4 = unconstrained MHE
estimatorOption = 1;

% turn process disturbance on or off
process_disturbance_on = 1; % 0 is off, 1 is on

sensor_disturbance_on = 0;

%create a thread-pool of n_workers
parpool('local', n_workers);

pool = gcp('nocreate'); %ensures we are using the pool from 'parpool'
if isempty(pool)
    disp("No pool created! Make sure parpool is called!");
else
    poolsize = pool.NumWorkers;
    disp("Worker Size: "+ poolsize);
end



tic
f_schedules(1:mc_length) = parallel.FevalFuture;
for i = 1:mc_length
    %savedstats{i} = Simulation_Function(1, estimatorOption, 1, process_disturbance_on, sensor_disturbance_on);
    f_schedules(i) = parfeval(pool, @Simulation_Function, 1, estimatorOption, start_phase, process_disturbance_on, sensor_disturbance_on);
end

savedstats = cell(1,mc_length);
for idx = 1:mc_length
    [completed_idx, output] = fetchNext(f_schedules);
    savedstats{idx} = output;
    fprintf('Index %d finished\n', completed_idx);
end
toc

save('loaded_data/test');
delete(gcp('nocreate'))
clear