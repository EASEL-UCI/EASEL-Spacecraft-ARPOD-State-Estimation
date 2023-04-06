% Paper_Visuals.m
% -------------------------
%{
    Creates Graph Visualizations for Paper Submission and Software Cycle Development
    
    Current Graphs that will be made:
    =================================
        - Start/Finish Distribution
        - Example 3D Trajectory Plot
            - 3d
            - xz xy yz
        - xyz 2d visuals wrt time
        - fuel bar graph
        - estimation_time bar graph
        - Trajectory Special (normalized distance)
            - sum of squared (definitely)
            - delta fuel
            - estimation_ts
        - Bar Graphs
            - fuel
            - estimation_ts
              
%}


function empty = GraphPaper(graph_num)
    close all

    if graph_num == 1
        %case 1
        modif = "_d.mat";
        folder = "Graphs2/Base/";
        bar_length = 1000;
    elseif graph_num == 2
        %case 2
        modif = "_pd.mat";
        folder = "Graphs2/PD/";
        bar_length = 1000;
    elseif graph_num == 3
        %case3
        modif = "_pd2.mat";
        folder = "Graphs2/Start2/";
        bar_length = 300;
    else
        error('you done goofed');
    end

    % folder = "Graphs2/Start2/";
    % modif = "_d.mat";
    % NO DISTURBANCE GRAPHS (STARTED PHASE 1)
    load("loaded_data/EKF"+modif, "-mat", "savedstats");
    %    EKF
    figure;
    GraphUtil.graphInitialDistribution(savedstats);
    title("EKF Start");
    saveas(gcf,folder+'EKF_start.eps','epsc');
    figure;
    GraphUtil.graphFinalDistribution(savedstats);
    title("EKF Finish");
    saveas(gcf,folder+'EKF_finish.eps','epsc');

    title_ = "EKF";
    GraphUtil.graph2DViewMissions(savedstats,title_,folder,true,bar_length);
    saveas(gcf,folder+'EKF_2d.eps','epsc');


    % folder = "Graphs2/Start2/";
    % modif = "_d.mat";
    %    PF
    load("loaded_data/PF"+modif, "-mat", "savedstats");
    figure;
    GraphUtil.graphInitialDistribution(savedstats);
    title("PF Start");
    saveas(gcf,folder+'PF_start.eps','epsc');
    figure;
    GraphUtil.graphFinalDistribution(savedstats);
    title("PF Finish");
    saveas(gcf,folder+'PF_finish.eps','epsc');
    title_ = "PF";
    GraphUtil.graph2DViewMissions(savedstats,title_,folder,true,bar_length);

    % folder = "Graphs2/Start2/";
    % modif = "_d.mat";
    %    MHE
    load("loaded_data/MHE"+modif, "-mat", "savedstats");
    figure;
    GraphUtil.graphInitialDistribution(savedstats);
    title("MHE Start");
    saveas(gcf,folder+'MHE_start.eps','epsc');
    figure;
    GraphUtil.graphFinalDistribution(savedstats);
    title("MHE Finish");
    saveas(gcf,folder+'MHE_finish.eps','epsc');
    figure;
    title_ = "MHE";
    GraphUtil.graph2DViewMissions(savedstats,title_,folder,true,bar_length);

    % Bar Graphs
    % folder = "Graphs2/Start2/";
    % modif = "_d.mat";
    files = ["loaded_data/EKF"+modif, "loaded_data/PF"+modif, "loaded_data/MHE"+modif];
    figure;
    GraphUtil.graphFuelBar(files);
    set(gca,'xticklabel',["EKF","PF","MHE"]);
    ylabel('Fuel [Ns]');
    title("Fuel Consumption Ns Graph");
    saveas(gcf,folder+'Fuel.eps','epsc');

    figure;
    GraphUtil.graphFuelBarPhase(files,1);
    title("Fuel Graph Phase 1");
    ylabel('Fuel [Ns]');
    set(gca,'xticklabel',["EKF","PF","MHE"]);
    saveas(gcf,folder+'Fuel_phase1.eps','epsc');

    figure;
    GraphUtil.graphFuelBarPhase(files,2);
    title("Fuel Graph Phase 2");
    ylabel('Fuel [Ns]');
    set(gca,'xticklabel',["EKF","PF","MHE"]);
    saveas(gcf,folder+'Fuel_phase2.eps','epsc');

    figure;
    GraphUtil.graphFuelBarPhase(files,3);
    title("Fuel Graph Phase 3");
    ylabel('Fuel [Ns]');
    set(gca,'xticklabel',["EKF","PF","MHE"]);
    saveas(gcf,folder+'Fuel_phase3.eps','epsc');


    figure;
    GraphUtil.graphRuntimeBar(files);
    title("Runtime Graph");
    ylabel('Runtime [ms]')
    set(gca,'xticklabel',["EKF","PF","MHE"]);
    saveas(gcf,folder+'Runtime.eps','epsc');


    figure;
    GraphUtil.graphRuntimeBarPhase(files,1);
    title("Runtime Graph Phase 1");
    ylabel('Runtime [ms]');
    set(gca,'xticklabel',["EKF","PF","MHE"]);
    saveas(gcf,folder+'Runtime_phase1.eps','epsc');

    figure;
    GraphUtil.graphRuntimeBarPhase(files,2);
    title("Runtime Graph Phase 2");
    ylabel('Runtime [ms]');
    set(gca,'xticklabel',["EKF","PF","MHE"]);
    saveas(gcf,folder+'Runtime_phase2.eps','epsc');

    figure;
    GraphUtil.graphRuntimeBarPhase(files,3);
    title("Runtime Graph Phase 3");
    ylabel('Runtime [ms]');
    set(gca,'xticklabel',["EKF","PF","MHE"]);
    saveas(gcf,folder+'Runtime_phase3.eps','epsc');

    figure;
    GraphUtil.DockSuccessBar(files,true);
    title("RMSE Pos Error Graph");
    ylabel('RMSE Pos Error');
    set(gca,'xticklabel',["EKF","PF","MHE"]);
    saveas(gcf,folder+'SoS.eps','epsc');

    figure;
    GraphUtil.ErrorPhaseBar(files,1,true);
    title("RMSE Pos Error Graph Phase 1");
    ylabel('RMSE Pos Error [m]');
    set(gca,'xticklabel',["EKF","PF","MHE"]);
    saveas(gcf,folder+'SoS_phase1.eps','epsc');

    figure;
    GraphUtil.ErrorPhaseBar(files,2,true);
    title("RMSE Pos Error Graph Phase 2");
    ylabel('RMSE Pos Error [m]');
    set(gca,'xticklabel',["EKF","PF","MHE"]);
    saveas(gcf,folder+'SoS_phase2.eps','epsc');

    figure;
    GraphUtil.ErrorPhaseBar(files,3,true);
    title("RMSE Pos Error Graph Phase 3");
    ylabel('RMSE Pos Error [m]');
    set(gca,'xticklabel',["EKF","PF","MHE"]);
    saveas(gcf,folder+'SoS_phase3.eps','epsc');

    figure;
    GraphUtil.DockSuccessBar(files,false);
    title("RMSE Vel Error Graph");
    ylabel('RMSE Error [m]');
    set(gca,'xticklabel',["EKF","PF","MHE"]);
    saveas(gcf,folder+'SoS_vel.eps','epsc');

    figure;
    GraphUtil.ErrorPhaseBar(files,1,false);
    title("RMSE Vel Error Graph Phase 1");
    ylabel('RMSE Vel Error');
    set(gca,'xticklabel',["EKF","PF","MHE"]);
    saveas(gcf,folder+'SoS_phase1_vel.eps','epsc');

    figure;
    GraphUtil.ErrorPhaseBar(files,2,false);
    title("RMSE Vel Error Graph Phase 2");
    ylabel('RMSE Vel Error');
    set(gca,'xticklabel',["EKF","PF","MHE"]);
    saveas(gcf,folder+'SoS_phase2_vel.eps','epsc');

    figure;
    GraphUtil.ErrorPhaseBar(files,3,false);
    title("RMSE Vel Error Graph Phase 3");
    ylabel('RMSE Vel Error');
    set(gca,'xticklabel',["EKF","PF","MHE"]);
    saveas(gcf,folder+'SoS_phase3_vel.eps','epsc');

    
end