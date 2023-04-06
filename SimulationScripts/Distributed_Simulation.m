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

function nothing =  Distributed_Simulation(estimator,cases)
    estimatorOption = estimator;
    sensor_disturbance_on = 0;
    if (estimatorOption == 1)
        file = "loaded_data/EKF";
    elseif (estimatorOption == 2)
        file = "loaded_data/PF";
    elseif (estimatorOption == 4)
        file = "loaded_data/MHE";
    else
        error("STOP!");
    end
    if (cases == 1)
        start_phase = 1;
        process_disturbance_on = 0;
        file = file + "_d.mat";
        disp(file);
    elseif (cases == 2)
        start_phase = 1;
        process_disturbance_on = 1;
        file = file + "_pd.mat";
        disp(file);
    elseif (cases == 3)
        start_phase = 2;
        process_disturbance_on = 1;
        file = file + "_pd2.mat";
        disp(file);
    end
    delete(gcp('nocreate'))

    n_workers = 7;
    mc_length = 100;

    % 1 = EKF
    % 2 = PF
    % 3 = constrained MHE
    % 4 = unconstrained MHE

    % turn process disturbance on or off
    % process_disturbance_on = 1; % 0 is off, 1 is on

    % sensor_disturbance_on = 1;

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
        f_schedules(i) = parfeval(pool, @Simulation_Function, 1, estimatorOption, start_phase, process_disturbance_on, sensor_disturbance_on);
    end

    savedstats = cell(1,mc_length);
    for idx = 1:mc_length
        [completed_idx, output] = fetchNext(f_schedules);
        savedstats{idx} = output;
        fprintf('Index %d finished\n', completed_idx);
    end
    toc
        
    save(file);
    delete(gcp('nocreate'))
    clear
end