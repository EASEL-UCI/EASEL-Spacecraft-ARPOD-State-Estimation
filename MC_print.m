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