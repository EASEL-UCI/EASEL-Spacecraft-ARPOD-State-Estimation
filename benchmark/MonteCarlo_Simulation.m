
monteCarlo_Length = 50;

start_phase = 1;


total_steps = 0;
times = [];
phase1Time = [];
phase2Time = [];
phase3Time = [];

phase1Error = [];
phase2Error = [];
phase3Error = [];
error = [];

estimatorOption = 3;

for i = 1:monteCarlo_Length
    disp("Monte Carlo Iteration: " + i)
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
    disp("Initial  Trajectory: [" + x + "," + y + "," + z + "," + vx + "," + vy + "," + vz + "]")
    stats = Benchmark([x;y;z;vx;vy;vz],estimatorOption,0*[1,1,1,1e-20,1e-20,1e-20],0,0);

    idxPhase1 = find(stats.trackPhase==1);
    idxPhase2 = find(stats.trackPhase==2);
    idxPhase3 = find(stats.trackPhase==3);
    Error = sum((stats.trackTraj - stats.trackEstTraj).^2)/6;
    Time = stats.estimation_ts;

    error = [error, Error];
    phase1Error = [phase1Error, Error(idxPhase1)];
    phase2Error = [phase2Error, Error(idxPhase2)];
    phase3Error = [phase3Error, Error(idxPhase3)];

    phase1Time = [phase1Time, Time(idxPhase1)];
    phase2Time = [phase2Time, Time(idxPhase2)];
    phase3Time = [phase3Time, Time(idxPhase3)];
    times = [times, stats.estimation_ts];
    total_steps = total_steps + stats.total_steps;
end
disp("Error Stats")
errorMean = mean(error);
errorMax = max(error);
errorMin = min(error);
errorStd = std(error);
disp("      errorMean:" + errorMean)
disp("      errorMax:" + errorMax)
disp("      errorMin:" + errorMin)
disp("      errorStd:" + errorStd)

disp("Phase 1 Error Stats")
errorMean = mean(phase1Error);
errorMax = max(phase1Error);
errorMin = min(phase1Error);
errorStd = std(phase1Error);
disp("      error1Mean:" + errorMean)
disp("      error1Max:" + errorMax)
disp("      error1Min:" + errorMin)
disp("      error1Std:" + errorStd)

disp("Phase 2 Error Stats")
errorMean = mean(phase2Error);
errorMax = max(phase2Error);
errorMin = min(phase2Error);
errorStd = std(phase2Error);
disp("      error2Mean:" + errorMean)
disp("      error2Max:" + errorMax)
disp("      error2Min:" + errorMin)
disp("      error2Std:" + errorStd)

disp("Phase 3 Error Stats")
errorMean = mean(phase3Error);
errorMax = max(phase3Error);
errorMin = min(phase3Error);
errorStd = std(phase3Error);
disp("      error3Mean:" + errorMean)
disp("      error3Max:" + errorMax)
disp("      error3Min:" + errorMin)
disp("      error3Std:" + errorStd)


disp("Phase 1 Time Stats")
timeMean = sum(phase1Time) / length(phase1Time);
timeMax = max(phase1Time);
timeMin = min(phase1Time);
timeStd = std(phase1Time);
disp("      timeMean:" + timeMean)
disp("      timeMax:" + timeMax)
disp("      timeMin:" + timeMin)
disp("      timeStd:" + timeStd)

disp("Phase 2 Time Stats")
timeMean = sum(phase2Time) / length(phase2Time);
timeMax = max(phase2Time);
timeMin = min(phase2Time);
timeStd = std(phase2Time);
disp("      timeMean:" + timeMean)
disp("      timeMax:" + timeMax)
disp("      timeMin:" + timeMin)
disp("      timeStd:" + timeStd)

disp("Phase 3 Time Stats")
timeMean = sum(phase3Time) / length(phase3Time);
timeMax = max(phase3Time);
timeMin = min(phase3Time);
timeStd = std(phase3Time);
disp("      timeMean:" + timeMean)
disp("      timeMax:" + timeMax)
disp("      timeMin:" + timeMin)
disp("      timeStd:" + timeStd)

timeMean = sum(times) / length(times);
timeMax = max(times);
timeMin = min(times);
timeStd = std(times);
disp("Time Stats")
disp("      timeMean:" + timeMean)
disp("      timeMax:" + timeMax)
disp("      timeMin:" + timeMin)
disp("      timeStd:" + timeStd)