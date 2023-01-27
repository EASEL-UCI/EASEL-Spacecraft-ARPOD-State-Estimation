
monteCarlo_Length = 50;

start_phase = 1;


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
estimatorOption = 4;

% turn process disturbance on or off
process_disturbance_on = 1; % 0 is off, 1 is on

sensor_disturbance_on = 1;

if process_disturbance_on
    process_disturbance = 1e-5*[1,1,1,1e-20,1e-20,1e-20];
else
    process_disturbance = 0*[1,1,1,1e-20,1e-20,1e-20];
end

if sensor_disturbance_on
    sensor_disturbance = 1e-3*[1,1,1];
else
    sensor_disturbance = 0*[1,1,1];
end

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
    
    
        
    stats = Benchmark([x;y;z;vx;vy;vz],estimatorOption,process_disturbance,0,sensor_disturbance);

    idxPhase1 = find(stats.trackPhase==1);
    idxPhase2 = find(stats.trackPhase==2);
    idxPhase3 = find(stats.trackPhase==3);
    
    error = [error, sum((stats.trackTraj - stats.trackEstTraj).^2)/6];
    
    XError = [XError, sum((stats.trackTraj(1:3,:) - stats.trackEstTraj(1:3,:)).^2)/3];
    
    XdotError = [XdotError, sum((stats.trackTraj(4:6,:) - stats.trackEstTraj(4:6,:)).^2)/3];
    
    Time = stats.estimation_ts;
    
    h = 1; % 1 s time step
    h*stats.total_steps;
    
    
    FuelConsumption = stats.trackFuelConsumption(end);
    
    
    phase1Error = error(idxPhase1);
    phase2Error = error(idxPhase2);
    phase3Error = error(idxPhase3);
    
    phase1XError = XError(idxPhase1);
    phase2XError = XError(idxPhase2);
    phase3XError = XError(idxPhase3);
    
    phase1XdotError = XdotError(idxPhase1);
    phase2XdotError = XdotError(idxPhase2);
    phase3XdotError = XdotError(idxPhase3);
    

    phase1Time = Time(idxPhase1);
    phase2Time = Time(idxPhase2);
    phase3Time = Time(idxPhase3);
    times = stats.estimation_ts;
%     total_steps = total_steps + stats.total_steps;
    
    savestats{i} = stats;
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






disp("XError Stats")
XerrorMean = mean(XError);
XerrorMax = max(XError);
XerrorMin = min(XError);
XerrorStd = std(XError);
disp("      errorMean:" + XerrorMean)
disp("      errorMax:" + XerrorMax)
disp("      errorMin:" + XerrorMin)
disp("      errorStd:" + XerrorStd)

disp("Phase 1 XError Stats")
XerrorMeanp1 = mean(phase1XError);
XerrorMaxp1 = max(phase1XError);
XerrorMinp1 = min(phase1XError);
XerrorStdp1 = std(phase1XError);
disp("      error1Mean:" + XerrorMeanp1)
disp("      error1Max:" + XerrorMaxp1)
disp("      error1Min:" + XerrorMinp1)
disp("      error1Std:" + XerrorStdp1)

disp("Phase 2 XError Stats")
XerrorMeanp2 = mean(phase2XError);
XerrorMaxp2 = max(phase2XError);
XerrorMinp2 = min(phase2XError);
XerrorStdp2 = std(phase2XError);
disp("      error2Mean:" + XerrorMeanp2)
disp("      error2Max:" + XerrorMaxp2)
disp("      error2Min:" + XerrorMinp2)
disp("      error2Std:" + XerrorStdp2)

disp("Phase 3 XError Stats")
XerrorMeanp3 = mean(phase3XError);
XerrorMaxp3 = max(phase3XError);
XerrorMinp3 = min(phase3XError);
XerrorStdp3 = std(phase3XError);
disp("      error3Mean:" + XerrorMeanp3)
disp("      error3Max:" + XerrorMaxp3)
disp("      error3Min:" + XerrorMinp3)
disp("      error3Std:" + XerrorStdp3)







disp("XdotError Stats")
XdoterrorMean = mean(XdotError);
XdoterrorMax = max(XdotError);
XdoterrorMin = min(XdotError);
XdoterrorStd = std(XdotError);
disp("      errorMean:" + XdoterrorMean)
disp("      errorMax:" + XdoterrorMax)
disp("      errorMin:" + XdoterrorMin)
disp("      errorStd:" + XdoterrorStd)

disp("Phase 1 XdotError Stats")
XdoterrorMeanp1 = mean(phase1XdotError);
XdoterrorMaxp1 = max(phase1XdotError);
XdoterrorMinp1 = min(phase1XdotError);
XdoterrorStdp1 = std(phase1XdotError);
disp("      error1Mean:" + XdoterrorMeanp1)
disp("      error1Max:" + XdoterrorMaxp1)
disp("      error1Min:" + XdoterrorMinp1)
disp("      error1Std:" + XdoterrorStdp1)

disp("Phase 2 XdotError Stats")
XdoterrorMeanp2 = mean(phase2XdotError);
XdoterrorMaxp2 = max(phase2XdotError);
XdoterrorMinp2 = min(phase2XdotError);
XdoterrorStdp2 = std(phase2XdotError);
disp("      error2Mean:" + XdoterrorMeanp2)
disp("      error2Max:" + XdoterrorMaxp2)
disp("      error2Min:" + XdoterrorMinp2)
disp("      error2Std:" + XdoterrorStdp2)

disp("Phase 3 XdotError Stats")
XdoterrorMeanp3 = mean(phase3XdotError);
XdoterrorMaxp3 = max(phase3XdotError);
XdoterrorMinp3 = min(phase3XdotError);
XdoterrorStdp3 = std(phase3XdotError);
disp("      error3Mean:" + XdoterrorMeanp3)
disp("      error3Max:" + XdoterrorMaxp3)
disp("      error3Min:" + XdoterrorMinp3)
disp("      error3Std:" + XdoterrorStdp3)




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

disp("Mission Time stats")
timeMean = sum(times) / length(times);
timeMax = max(times);
timeMin = min(times);
timeStd = std(times);
disp("Time Stats")
disp("      timeMean:" + timeMean)
disp("      timeMax:" + timeMax)
disp("      timeMin:" + timeMin)
disp("      timeStd:" + timeStd)

disp("Fuel Consumption stats")
FuelConsumptionMean = sum(FuelConsumption) / length(FuelConsumption);
FuelConsumptionMax = max(FuelConsumption);
FuelConsumptionMin = min(FuelConsumption);
FuelConsumptionStd = std(FuelConsumption);
disp("Time Stats")
disp("      timeMean:" + FuelConsumptionMean)
disp("      timeMax:" + FuelConsumptionMax)
disp("      timeMin:" + FuelConsumptionMin)
disp("      timeStd:" + FuelConsumptionStd)


save("test");