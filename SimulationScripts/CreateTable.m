
%{

\begin{table}
\caption{Estimation performance for Case 1 with measurement noise $\vv_t$. $E(\cdot)$ denotes the root-mean-squared-error. c.t. and m. t. denote ``computation time'' at each time step and ``mission time,'' respectively.}
\label{tab:noise-performance}
\centering
\footnotesize
\begin{tabular}{@{} | c | c | c  c  c | c  c  c | c  c  c | c  c  c |}
\multicolumn{2}{c}{} & \multicolumn{3}{c}{Phase 1} & \multicolumn{3}{c}{Phase 2} & \multicolumn{3}{c}{Phase 3} & \multicolumn{3}{c}{Entire Mission}\\
\hline
 Metric & Units & EKF & PF & MHE & EKF & PF & MHE & EKF & PF & MHE & EKF & PF & MHE \\
\hline
mean\{E$(\x$)\} & m & 3.4e-5 & 2.7e-5 & 2.5e-5 & 5.6e-5 & 4.2e-5 & 5.6e-5 & 9.9e-5 & 7.4e-5  & 6.2e-5  & 2.5e-4 & 1.1e-4 & 1.4e-4\\
stdev\{E($\x$)\} & m & 2.7e-5 & 2.1e-5 & 2.9e-5 & 3.0e-5 & 2.3e-5 & 3.3e-6 & 7.6e-7 & 5.2e-7  & 6.6e-7  & 0.002 & 1.2e-4 & 1.5e-4 \\
max\{E$(\x$)\} & m & 8.0e-5 & 6.3e-5 & 2.0e-4 & 9.8e-5 & 7.3e-5 & 6.1e-5 & 1.0e-4  & 7.5e-5  & 6.3e-5  & 0.04  & 6.6e-4 & 1.3e-3  \\
min\{E$(\x$)\} & m & 0       &  0 & 0      & 2.1e-5 & 1.7e-5 & 5.0e-5 & 9.8e-5     & 7.3e-5  & 6.2e-5  & 0     & 0   & 0  \\
\hline
mean\{E$(\dot\x$)\} & m/s & 2.1e-7   & 1.5e-7  & 7.4e-7 & 2.7e-7 & 1.9e-7 & 4.2e-7 & 3.1e-7  & 2.2e-7 & 4.2e-7 & 7.5e-7 & 3.5e-7 & 8.0e-7\\
stdev\{E($\dot\x$)\} & m/s & 8.0e-8  & 5.4e-8 & 1.7e-6  & 3.5e-8 & 2.1e-8 & 3.4e-10 & 1.6e-11 & 2.7e-10 & 2.5e-11& 6.0e-6 & 2.4e-7 & 3.0e-6 \\
max\{E$(\dot\x$)\} & m/s  & 3.0e-7  & 2.1e-7 & 1.1e-5  & 3.1e-7 & 2.2e-7 & 4.2e-7 & 3.1e-7    & 2.2e-7 & 4.2e-7 & 1.0e-4 & 1.0e-6 & 7.1e-5\\
min\{E$(\dot\x$)\} & m/s  & 0       & 0 & 0      & 2.4e-7       & 1.7e-7 & 4.2e-7 & 3.1e-7    & 2.2e-7 & 4.2e-7 & 0      & 0 & 0 \\
\hline
mean\{c. t.\} & s  & 6.0e-5  & 0.044 & 0.017& 5.7e-7 & 0.041 & 0.019 & 5.7e-5   & 0.041 & 0.019 & 5.9e-5 & 0.043 & 0.017\\
stdev\{c. t.\} & s & 5.7e-5  & 0.003 & 0.014 & 2.2e-4 & 0.002 & 9.5e-4 & 1.8e-5 & 0.002 & 9.2e-4 & 1.3e-4 & 0.003 & 0.012 \\
max\{c. t.\} & s   & 6.1e-3  & 0.122 & 2.05 & 0.022 & 0.138 & 0.028 & 4.6e-4   & 0.057 & 0.025 & 0.022 & 0.138 & 2.05\\
min\{c. t.\} & s   & 0       & 0     & 0 & 4.5e-5    & 0.038 & 0.017 & 4.7e-5  & 0.039 & 0.018 & 0 & 0 & 0 \\
\hline
mean\{m. t.\} & s & - & - & - & - & - & - & - & - & - & 614 & 574 & 627 \\
stdev\{m. t.\} & s & - & - & - & - & - & - & - & - & - & 186 & 196 & 202 \\
max\{m. t.\} & s & - & - & - & - & - & - & - & - & - & 1013 & 921  & 1033 \\
min\{m. t.\} & s & - & - & - & - & - & - & - & - & - & 203 & 157 & 197 \\
\hline
mean\{fuel\} & Ns & - & - & - & - & - & - & - & - & -  & 151 & 139 & 155 \\
stdev\{fuel\} & Ns & - & - & - & - & - & - & - & - & - & 47.4 & 46.6 & 47.9 \\
max\{fuel\} & Ns & - & - & - & - & - & - & - & - & -   & 263 & 223 & 274 \\
min\{fuel\} & Ns & - & - & - & - & - & - & - & - & -   & 39 & 36.4 & 45.3  \\
\hline
\end{tabular}
\end{table}
%}

function table_str = CreateTable(graph_num)
    file = "loaded_data/";

    if graph_num == 1
        modif = "_d.mat";
        phase2_start_bool = false;
    elseif graph_num == 2
        modif = "_pd.mat";
        phase2_start_bool = false;
    elseif graph_num == 3
        modif = "_pd2.mat";
        phase2_start_bool = true;
    else
        error('OOF');
    end
    list_name_stats = [file+"EKF"+modif, file+"PF"+modif, file+"MHE"+modif];
    %the meat of the table
    idxEntirePhase1 = cell(3,1);
    idxEntirePhase2 = cell(3,1);
    idxEntirePhase3 = cell(3,1);

    XError = cell(3,1);
    XdotError = cell(3,1);

    mission_times = cell(3,1);
    fuelConsumption = cell(3,1);
    times = cell(3,1);
    for m = 1:length(list_name_stats)
        load(list_name_stats(m));%loading up
        XError{m} = [];
        XdotError{m} = [];
        mission_times{m} = [];
        fuelConsumption{m} = [];
        idxEntirePhase1{m} = [];
        idxEntirePhase2{m} = [];
        idxEntirePhase3{m} = [];

        phases = [];    
        times{m} = [];
        episodes = length(savedstats);
        for j = 1:episodes
            stats = savedstats{j};
            phases = [phases, stats.trackPhase];
            
            % mse = ((stats.trackTraj-stats.trackEstTraj).^2)/3;
            gt = stats.trackTraj;
            est = stats.trackEstTraj;

            phase_ = stats.trackPhase;
            mse_pos = sqrt(mean(vecnorm(gt(1:3,:) - est(1:3,:))).^2);
            mse_vel = sqrt(mean(vecnorm(gt(4:6,:) - est(4:6,:))).^2);

            mse_pos1 = sqrt(mean(vecnorm(gt(1:3,phase_==1) - est(1:3,phase_==1))).^2);
            mse_vel1 = sqrt(mean(vecnorm(gt(4:6,phase_==1) - est(4:6,phase_==1))).^2);

            mse_pos2 = sqrt(mean(vecnorm(gt(1:3,phase_==2) - est(1:3,phase_==2))).^2);
            mse_vel2 = sqrt(mean(vecnorm(gt(4:6,phase_==2) - est(4:6,phase_==2))).^2);

            mse_pos3 = sqrt(mean(vecnorm(gt(1:3,phase_==3) - est(1:3,phase_==3))).^2);
            mse_vel3 = sqrt(mean(vecnorm(gt(4:6,phase_==3) - est(4:6,phase_==3))).^2);

            stack = [mse_pos; mse_pos1; mse_pos2;mse_pos3];
            stack_vel = [mse_vel; mse_vel1; mse_vel2; mse_vel3];
            XdotError{m} = [XdotError{m}, stack_vel];
            XError{m} = [XError{m}, stack];
        
            times{m} = [times{m}, stats.estimation_ts];
            mission_times{m} = [mission_times{m}, stats.timestamps(end)];
            fuelConsumption{m} = [fuelConsumption{m}, stats.trackFuelConsumption(end)];
        end
        % XError{m} = XError{m};
        % XdotError{m} = XdotError{m};
        % XError -> [stack, stack,....] stack = [error,errorp1,e]

        idxEntirePhase1{m} = find(phases==1);
        idxEntirePhase2{m} = find(phases==2);
        idxEntirePhase3{m} = find(phases==3);
    end
    numbers_str = "";
    numbers_str = numbers_str + "\\hline\n";
    numbers_str = numbers_str + "mean\\{E$(\\x$)\\} & m ";
    % numbers_str = numbers_str + Helper(XError, idxEntirePhase1,idxEntirePhase2,idxEntirePhase3,@mean, false);
    numbers_str = numbers_str + Helper2(XError, @mean);
    
    numbers_str = numbers_str + "stdev\\{E$(\\x$)\\} & m ";
    % numbers_str = numbers_str + Helper(XError, idxEntirePhase1,idxEntirePhase2,idxEntirePhase3,@std, false);
    numbers_str = numbers_str + Helper2(XError, @std);
    numbers_str = numbers_str + "max\\{E$(\\x$)\\} & m ";
    % numbers_str = numbers_str + Helper(XError, idxEntirePhase1,idxEntirePhase2,idxEntirePhase3,@max, false);
    numbers_str = numbers_str + Helper2(XError, @max);
    numbers_str = numbers_str + "min\\{E$(\\x$)\\} & m ";
    % numbers_str = numbers_str + Helper(XError, idxEntirePhase1,idxEntirePhase2,idxEntirePhase3,@min, false);
    numbers_str = numbers_str + Helper2(XError, @min);

    numbers_str = numbers_str + "\\hline\n";

    numbers_str = numbers_str + "mean\\{E$(\\x$)\\} & $\\frac{m}{s}$ ";
    % numbers_str = numbers_str + Helper(XdotError, idxEntirePhase1,idxEntirePhase2,idxEntirePhase3,@mean, false);
    numbers_str = numbers_str + Helper2(XdotError, @mean);
    numbers_str = numbers_str + "stdev\\{E$(\\x$)\\} & $\\frac{m}{s}$ ";
    % numbers_str = numbers_str + Helper(XdotError, idxEntirePhase1,idxEntirePhase2,idxEntirePhase3,@std, false);
    numbers_str = numbers_str + Helper2(XdotError, @std);
    numbers_str = numbers_str + "max\\{E$(\\x$)\\} & $\\frac{m}{s}$ ";
    % numbers_str = numbers_str + Helper(XdotError, idxEntirePhase1,idxEntirePhase2,idxEntirePhase3,@max, false);
    numbers_str = numbers_str + Helper2(XdotError, @max);
    numbers_str = numbers_str + "min\\{E$(\\x$)\\} & $\\frac{m}{s}$ ";
    % numbers_str = numbers_str + Helper(XdotError, idxEntirePhase1,idxEntirePhase2,idxEntirePhase3,@min, false);
    numbers_str = numbers_str + Helper2(XdotError, @min);
    numbers_str = numbers_str + "\\hline\n";

    numbers_str = numbers_str + "mean\\{c.t.\\} & s ";
    numbers_str = numbers_str + Helper(times, idxEntirePhase1,idxEntirePhase2,idxEntirePhase3,@mean, false);
    numbers_str = numbers_str + "stdev\\{c.t.\\} & s ";
    numbers_str = numbers_str + Helper(times, idxEntirePhase1,idxEntirePhase2,idxEntirePhase3,@std, false);
    numbers_str = numbers_str + "max\\{c.t.\\} & s ";
    numbers_str = numbers_str + Helper(times, idxEntirePhase1,idxEntirePhase2,idxEntirePhase3,@max, false);
    numbers_str = numbers_str + "min\\{c.t.\\} & s ";
    numbers_str = numbers_str + Helper(times, idxEntirePhase1,idxEntirePhase2,idxEntirePhase3,@min, false);

    numbers_str = numbers_str + "\\hline\n";

    numbers_str = numbers_str + "mean\\{m.t.\\} & s ";
    numbers_str = numbers_str + Helper(mission_times, idxEntirePhase1,idxEntirePhase2,idxEntirePhase3,@mean, true);
    numbers_str = numbers_str + "stdev\\{m.t.\\} & s ";
    numbers_str = numbers_str + Helper(mission_times, idxEntirePhase1,idxEntirePhase2,idxEntirePhase3,@std, true);
    numbers_str = numbers_str + "max\\{m.t.\\} & s ";
    numbers_str = numbers_str + Helper(mission_times, idxEntirePhase1,idxEntirePhase2,idxEntirePhase3,@max, true);
    numbers_str = numbers_str + "min\\{m.t.\\} & s ";
    numbers_str = numbers_str + Helper(mission_times, idxEntirePhase1,idxEntirePhase2,idxEntirePhase3,@min, true);

    numbers_str = numbers_str + "\\hline\n";

    numbers_str = numbers_str + "mean\\{fuel\\} & Ns ";
    numbers_str = numbers_str + Helper(fuelConsumption, idxEntirePhase1,idxEntirePhase2,idxEntirePhase3,@mean, true);
    numbers_str = numbers_str + "stdev\\{fuel\\} & Ns ";
    numbers_str = numbers_str + Helper(fuelConsumption, idxEntirePhase1,idxEntirePhase2,idxEntirePhase3,@std, true);
    numbers_str = numbers_str + "max\\{fuel\\} & Ns ";
    numbers_str = numbers_str + Helper(fuelConsumption, idxEntirePhase1,idxEntirePhase2,idxEntirePhase3,@max, true);
    numbers_str = numbers_str + "min\\{fuel\\} & Ns ";
    numbers_str = numbers_str + Helper(fuelConsumption, idxEntirePhase1,idxEntirePhase2,idxEntirePhase3,@min, true);

    numbers_str = numbers_str + "\\hline\n";

    table_str = numbers_str;
    %table_str = table_str + numbers_str;
    % str_num = round(a/10.^(floor(log(a)/log(10))),1)+"e"+floor(log(a)/log(10));
    %end

end