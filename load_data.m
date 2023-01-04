clear
clc
load("EKF_d.mat")

episodes = length(savestats);

phases = [];
XError = [];
XdotError = [];
times = [];
mission_times = [];
fuelConsumption = [];
for i = 1:episodes
    stats = savestats{i};
    phases = [phases, stats.trackPhase];
    
    mse = ((stats.trackTraj-stats.trackEstTraj).^2)/3;
    XdotError = [XdotError, sum(mse(4:6,:))];
    XError = [XError, sum(mse(1:3,:)) ];

    times = [times, stats.estimation_ts];
    mission_times = [mission_times, stats.timestamps(end)];
    fuelConsumption = [fuelConsumption, stats.trackFuelConsumption(end)];
end

XError = sqrt(XError * 1e6);
XdotError = sqrt(XdotError * 1e6);

idxEntirePhase1 = find(phases==1);
idxEntirePhase2 = find(phases==2);
idxEntirePhase3 = find(phases==3);

phase1Error = XError(idxEntirePhase1);
phase2Error = XError(idxEntirePhase2);
phase3Error = XError(idxEntirePhase3);

phase1XdotError = XdotError(idxEntirePhase1);
phase2XdotError = XdotError(idxEntirePhase2);
phase3XdotError = XdotError(idxEntirePhase3);

phase1Time = times(idxEntirePhase1);
phase2Time = times(idxEntirePhase2);
phase3Time = times(idxEntirePhase3);

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

timeMean = sum(times) / length(times);
timeMax = max(times);
timeMin = min(times);
timeStd = std(times);
disp("Time Stats")
disp("      timeMean:" + timeMean)
disp("      timeMax:" + timeMax)
disp("      timeMin:" + timeMin)
disp("      timeStd:" + timeStd)

disp("Fuel Stats")
fuelMean = mean(fuelConsumption);
fuelMax = max(fuelConsumption);
fuelMin = min(fuelConsumption);
fuelStd = std(fuelConsumption);
disp("      fuelMean:" + fuelMean/500)
disp("      fuelMax:" + fuelMax/500)
disp("      fuelMin:" + fuelMin/500)
disp("      fuelStd:" + fuelStd/500)

disp("Mission Time Stats")
missionMean= mean(mission_times);
missionMax = max(mission_times);
missionMin = min(mission_times);
missionStd = std(mission_times);
disp("      missionMean:" + missionMean)
disp("      missionMax:" + missionMax)
disp("      missionMin:" + missionMin)
disp("      missionStd:" + missionStd)

save('PF_2');