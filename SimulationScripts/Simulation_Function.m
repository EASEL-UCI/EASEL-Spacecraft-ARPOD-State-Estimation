

function savestats = Simulation_Function(estimatorOption, start_phase, process_disturbance_on, sensor_disturbance_on)
    % total_steps = 0;
    times = [];

    phase1Time = [];
    phase2Time = [];
    phase3Time = [];

    phase1Error = [];
    phase2Error = [];
    phase3Error = [];
    error = [];


    phase1XError = [];
    phase2XError = [];
    phase3XError = [];
    XError = [];

    phase1XdotError = [];
    phase2XdotError = [];
    phase3XdotError = [];
    XdotError = [];

    FuelConsumption = [];

    % 1 = EKF
    % 2 = PF
    % 3 = constrained MHE
    % 4 = unconstrained MHE
    %estimatorOption = 4;

    % turn process disturbance on or off
    %process_disturbance_on = 1; % 0 is off, 1 is on

    %sensor_disturbance_on = 1;
    if (process_disturbance_on == 1)
        process_disturbance = 1e-5*[1,1,1,1e-20,1e-20,1e-20];
    else
        process_disturbance = 0*[1,1,1,1e-20,1e-20,1e-20];
    end

    if (sensor_disturbance_on == 1)
        sensor_disturbance = 1e-3*[1,1,1];
    else
        sensor_disturbance = 0*[1,1,1];
    end

    if start_phase == 1
        x = unifrnd(-5,5);
        y = unifrnd(-5,5);
        z = unifrnd(-5,5);
        vx = unifrnd(-1e-3,1e-3);
        vy = unifrnd(-1e-3,1e-3);
        vz = unifrnd(-1e-3,1e-3);

        while sqrt(x*x + y*y + z*z) < 2*ARPOD_Benchmark.rho_r
            x = unifrnd(-5,5);
            y = unifrnd(-5,5);
            z = unifrnd(-5,5);
        end
    elseif start_phase == 2
        x = unifrnd(-1,1);
        y = unifrnd(-1,1);
        z = unifrnd(-1,1);

        vx = unifrnd(-1e-5,1e-5);
        vy = unifrnd(-1e-5,1e-5);
        vz = unifrnd(-1e-5,1e-5);
        while sqrt(x*x + y*y + z*z) < ARPOD_Benchmark.rho_d
            x = unifrnd(-1,1);
            y = unifrnd(-1,1);
            z = unifrnd(-1,1);
        end
    elseif start_phase == 3
        x = unifrnd(-0.5,0.5);
        y = unifrnd(-0.5,0.5);
        z = unifrnd(-0.5,0.5);
        while sqrt(x*x + y*y + z*z) > ARPOD_Benchmark.rho_d
            x = unifrnd(-0.5,0.5);
            y = unifrnd(-0.5,0.5);
            z = unifrnd(-0.5,0.5);
        end
        vx = unifrnd(-1e-7,1e-7);
        vy = unifrnd(-1e-7,1e-7);
        vz = unifrnd(-1e-7,1e-7);
    end
     
    savestats = Benchmark([x;y;z;vx;vy;vz],estimatorOption,process_disturbance,0,sensor_disturbance);
end
