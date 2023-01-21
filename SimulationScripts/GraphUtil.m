
%{
    TODO:
    ------
    % graph 1000 trajectories for docking
    % graph translational state components
    % bar graphs

    % normalize along percent of completed path
    % or normalize along percent of time completed
%}

%{
    Graphs we'd use:
    ----------------
        - visualize trajectories
            - show (mean,stdev) along normalized paths
            - can show clean way?
        - plot what's in the tables as bar graphs
        
        - see if I can make a GIF (presentation)

    Extra notes:
    -------------
    - tables will go in appendix
        - distance, time, error, computation, fuel
    - convert each to bar graph
    - look into state estimation ARPOD papers.
        - just to get a good debrief to represent our current situation.
        - MHE. vs. PF
        - EKF. vs. PF
        - MHE vs. EKF
%}

classdef GraphUtil
    properties (Constant)
        theta1 = 60*pi/180;
        theta2 = GraphUtil.theta1;
    end
    methods (Static)

        %%%%% HELPER CODE %%%%%
        function void = setLatex()
            set(groot,'defaultAxesTickLabelInterpreter','latex');  
            set(groot,'defaulttextinterpreter','latex');
            set(groot,'defaultLegendInterpreter','latex');
        end
        function void = graphTarget()
            [x,y,z] = sphere(10);
            r = ARPOD_Benchmark.rho_r;
            scatter3(r*x(:),r*y(:),r*z(:), 5,'o', 'filled', 'MarkerEdgeColor', 'c');
            hold on
            [x,y,z] = sphere(5);
            r = ARPOD_Benchmark.rho_d;
            scatter3(r*x(:),r*y(:),r*z(:), 5, 'o', 'filled', 'MarkerEdgeColor', 'm');

            theta1 = 60 * pi / 180;
            theta2 = theta1;
            c = ARPOD_Benchmark.c.';
            plot3([0,-cos(theta1/2)*c], [0,-sin(theta2/2)*c], [0,-sin(theta1/2)*c], 'g');
            %plot3([0,-sin(theta2/2)*c], [0,sin(theta1/2)*c], [0,-cos(theta1/2)*c], 'g');
            plot3([0,-cos(theta1/2)*c], [0,sin(theta1/2)*c], [0,-sin(theta2/2)*c], 'g');

            %plot3([0,sin(theta2/2)*c], [0,-sin(theta1/2)*c], [0,-cos(theta1/2)*c], 'g');
            plot3([0,-cos(theta1/2)*c], [0,-sin(theta1/2)*c],[0,sin(theta2/2)*c],  'g');

            %plot3([0,sin(theta2/2)*c], [0,sin(theta1/2)*c], [0,-cos(theta1/2)*c], 'g');
            plot3([0,-cos(theta1/2)*c], [0,sin(theta1/2)*c], [0,sin(theta2/2)*c],  'g');

            %drawing base of pyramid
            %plot3([-sin(theta2/2)*c, sin(theta2/2)*c], [-sin(theta1/2)*c,-sin(theta1/2)*c], [-cos(theta1/2)*c,-cos(theta1/2)*c], 'g');
            plot3([-cos(theta1/2)*c,-cos(theta1/2)*c], [-sin(theta1/2)*c,-sin(theta1/2)*c],[-sin(theta2/2)*c, sin(theta2/2)*c], 'g');

            %plot3([-sin(theta2/2)*c, sin(theta2/2)*c], [sin(theta1/2)*c, sin(theta1/2)*c], [-cos(theta1/2)*c,-cos(theta1/2)*c], 'g');
            plot3([-cos(theta1/2)*c,-cos(theta1/2)*c],[sin(theta1/2)*c, sin(theta1/2)*c], [-sin(theta2/2)*c, sin(theta2/2)*c],'g');

            %plot3([-sin(theta2/2)*c, -sin(theta2/2)*c], [-sin(theta1/2)*c,sin(theta1/2)*c], [-cos(theta1/2)*c,-cos(theta1/2)*c], 'g');
            plot3([-cos(theta1/2)*c,-cos(theta1/2)*c], [-sin(theta1/2)*c,sin(theta1/2)*c], [-sin(theta2/2)*c, -sin(theta2/2)*c], 'g');

            %plot3([sin(theta2/2)*c, sin(theta2/2)*c], [-sin(theta1/2)*c, sin(theta1/2)*c], [-cos(theta1/2)*c,-cos(theta1/2)*c], 'g');
            plot3([-cos(theta1/2)*c,-cos(theta1/2)*c],[-sin(theta1/2)*c, sin(theta1/2)*c], [sin(theta2/2)*c, sin(theta2/2)*c], 'g');
            hold off
            xlabel('X [m]');
            ylabel('Y [m]');
            zlabel('Z [m]');
        end
        function [r,g,b] = randomize_color()
            [r,g,b] = [rand,rand,rand]
        end
        function void = graphPtDistribution(x,y,z,color, first_graph)
            %{
                x: list of x trajectories (1,length)
                y: list of y trajectories (1,length)
                z: list of z trajectories (1,length)
                color: char -> 'r' is red 'b' is blue 'bl' is black
            %}
            scatter3(x,y,z,color);
            if first_graph
                %axis equal
                hold on
            end
        end
        function void = graph2dPt(x,y,color,first_graph)
            scatter(x,y,color);
            if first_graph
                hold on
            end
        end
        function void = graphTrajectory(x,y,z,color,first_graph)
            plot3(x,y,z,color);
            if first_graph
                %axis equal
                hold on
            end
        end
        function void = graphTranslation(x,y,z,first_graph)
            t = length(x);
            ts = linspace(0,t,t);
            plot(ts,x,'r');
            if first_graph
                %axis equal
                hold on
            end
            plot(ts,y,'g');
            plot(ts,z,'b');

        end
        %%%%% END OF HELPER CODE %%%%%

        %%%%% START OF UTIL CODE %%%%%
        function void = graphInitialDistribution(arpod_stats)
            %{
                arpod_stats: Arpod_Statistics Data Structure array
            %}
            xs = zeros(length(arpod_stats));
            ys = zeros(length(arpod_stats));
            zs = zeros(length(arpod_stats));
            for idx = 1:length(arpod_stats)
                tracked_trajs = arpod_stats{idx}.trackTraj; %retrieve trajectories
                xs(idx) = tracked_trajs(1,1);
                ys(idx) = tracked_trajs(2,1);
                zs(idx) = tracked_trajs(3,1);
            end
            GraphUtil.graphPtDistribution(xs,ys,zs,'r', true);
            hold off
        end

        function void = graphFinalDistribution(arpod_stats)
            %{
                arpod_stats: Arpod_Statistics Data Structure array
            %}
            xs = zeros(length(arpod_stats));
            ys = zeros(length(arpod_stats));
            zs = zeros(length(arpod_stats));
            for idx = 1:length(arpod_stats)
                tracked_trajs = arpod_stats{idx}.trackTraj; %retrieve trajectories
                xs(idx) = tracked_trajs(1,end);
                ys(idx) = tracked_trajs(2,end);
                zs(idx) = tracked_trajs(3,end);
            end
            GraphUtil.graphPtDistribution(xs,ys,zs,'b', true);
            hold off
        end

        function void = graphStartFinishDistribution(arpod_stats)
            xs = zeros(length(arpod_stats));
            ys = zeros(length(arpod_stats));
            zs = zeros(length(arpod_stats));
            for idx = 1:length(arpod_stats)
                tracked_trajs = arpod_stats{idx}.trackTraj; %retrieve trajectories
                xs(idx) = tracked_trajs(1,end);
                ys(idx) = tracked_trajs(2,end);
                zs(idx) = tracked_trajs(3,end);
            end
            GraphUtil.graphPtDistribution(xs,ys,zs,'b', true);

            xs = zeros(length(arpod_stats));
            ys = zeros(length(arpod_stats));
            zs = zeros(length(arpod_stats));
            for idx = 1:length(arpod_stats)
                tracked_trajs = arpod_stats{idx}.trackTraj; %retrieve trajectories
                xs(idx) = tracked_trajs(1,1);
                ys(idx) = tracked_trajs(2,1);
                zs(idx) = tracked_trajs(3,1);
            end
            GraphUtil.graphPtDistribution(xs,ys,zs,'r', false);
            hold off
        end

        function void = graphTranslationDist(arpod_stats)
            first_graph = true;
            for idx = 1:length(arpod_stats)
                track_trajs = arpod_stats{idx}.trackTraj;
                xs = track_trajs(1,:);
                xs = xs(:);
                ys = track_trajs(2,:);
                ys = ys(:);
                zs = track_trajs(3,:);
                zs = zs(:);
                GraphUtil.graphTranslation(xs,ys,zs,first_graph);
                if first_graph
                    first_graph = false;
                end
            end
            hold off;
        end

        function void = graphTrajectoryDist(arpod_stats)
            first_graph = true;
            for idx = 1:length(arpod_stats)
                track_trajs = arpod_stats{idx}.trackTraj;
                xs = track_trajs(1,:);
                xs = xs(:);
                ys = track_trajs(2,:);
                ys = ys(:);
                zs = track_trajs(3,:);
                zs = zs(:);
                plot3(xs,ys,zs,'Color',[rand,rand,rand]);
                if first_graph
                    %axis equal
                    hold on 
                    first_graph = false;
                end
            end
            GraphUtil.graphTarget();
            hold off
        end

        function void = graph2DViewMission(arpod_stat)
            trackTraj = arpod_stat.trackTraj;
            trackU = [arpod_stat.trackU, arpod_stat.trackU(:,end)];
            xs = trackTraj(1,:);
            ys = trackTraj(2,:);
            zs = trackTraj(3,:);
            
            vxs = trackTraj(4,:);
            vys = trackTraj(5,:);
            vzs = trackTraj(6,:);

            uxs = trackU(1,:);
            uys = trackU(2,:);
            uzs = trackU(3,:);

            ts = arpod_stat.timestamps;

            subplot(3,1,1)
            plot(ts, xs, 'r');
            hold on
            plot(ts,ys,'g');
            plot(ts,zs,'b');
            hold off

            subplot(3,1,2)
            plot(ts, vxs, 'r');
            hold on
            plot(ts,vys,'g');
            plot(ts,vzs,'b');
            hold off

            subplot(3,1,3)
            plot(ts, uxs, 'r');
            hold on
            plot(ts,uys,'g');
            plot(ts,uzs,'b');
            hold off
        end

        function percentages = normalizeTrajectories(track_trajs)
            num_steps = length(track_trajs);
            distance = 0;
            percentages = zeros(1,num_steps);
            for idx = 2:num_steps
                %distance = distance + sqrt( sum( ((track_trajs(idx) - track_trajs(idx-1)).^2).' ) );
                distance = distance + norm(track_trajs(idx) - track_trajs(idx-1));
                percentages(idx) = distance;
            end
            percentages = percentages / distance;
        end
        function void = graphTrajectorySpecial(arpod_stats)
            first_graph = true;
            sum_trajs = 0;
            percent_trajs = [];
            errors_ys = [];
            for idx = 1:length(arpod_stats)
                track_trajs = arpod_stats{idx}.trackTraj;
                %normalize trajectories (get x axis stuff)
                percent_traj = GraphUtil.normalizeTrajectories(track_trajs);

                %get y axis stuff
                errors_y = sum( (arpod_stats{idx}.trackEstTraj - arpod_stats{idx}.trackTraj).^2 );

                percent_trajs = [percent_trajs, percent_traj];
                errors_ys = [errors_ys, errors_y];
            end
            GraphUtil.graph2dPt(percent_traj, errors_y,'r',first_graph);
            hold off
        end

        function void = graphFuelBar(list_name_stats)
            %this will create a boxcat graph of the fuel consumption
            
            list_fuel_stats = [];
            for idx = 1:length(list_name_stats)
                load(list_name_stats(idx));
                arpod_stats = savestats;
                length(arpod_stats)
                list_fuel_stats = [list_fuel_stats; zeros(1,length(arpod_stats))];
            end
            for idx = 1:length(list_name_stats)
                load(list_name_stats(idx));
                arpod_stats = savestats;
                for idx_idx = 1:length(arpod_stats)
                    list_fuel_stats(idx,idx_idx) = arpod_stats{idx_idx}.trackFuelConsumption(end);
                end
            end
            xrange = linspace(1,length(list_name_stats),length(list_name_stats));
            boxchart(list_fuel_stats.')
            
        end

        function void = graphRuntimeBar(list_name_stats)
            list_runtime_stats = [];
            for idx = 1:length(list_name_stats)
                load(list_name_stats(idx));
                arpod_stats = savestats;

                runtime_stats = [];
                for idx_idx = 1:length(arpod_stats)
                    runtime_stats = [runtime_stats, arpod_stats{idx_idx}.estimation_ts*1000];
                end
                isout = isoutlier(runtime_stats, 'quartiles');
                runtime_stats(isout) = NaN;
                if length(list_runtime_stats) > 0
                    if length(list_runtime_stats) > length(runtime_stats)
                        dist = length(list_runtime_stats) - length(runtime_stats);
                        runtime_stats = [runtime_stats, NaN(1,dist)];
                    elseif length(list_runtime_stats) < length(runtime_stats)
                        [row,col] = size(list_runtime_stats);
                        list_runtime_stats = [list_runtime_stats, NaN(row,length(runtime_stats)-col)];
                    end
                end
                list_runtime_stats = [list_runtime_stats; runtime_stats];
            end
            boxchart(list_runtime_stats.');
        end

        function void = DockSuccessBar(list_name_stats)

            success_rates = zeros(1,length(list_name_stats));
            for idx = 1:length(list_name_stats)
                load(list_name_stats(idx));
                arpod_stats = savestats;
                
                success_rate = 0.0;
                
                for idx_idx = 1:length(arpod_stats)
                    estTraj = arpod_stats{idx_idx}.trackTraj(1:3,end);
                    lastPhase = arpod_stats{idx_idx}.trackPhase(end);

                    if sqrt(estTraj(1).^2 + estTraj(2).^2 + estTraj(3).^2) < 1e-3
                        success_rate = success_rate + 1;
                    end
                end
                disp(success_rate)
                disp(length(arpod_stats))
                success_rate = double(success_rate) / double(length(arpod_stats));
                success_rates(idx) = success_rate;
            end
            bar(success_rates);
        end
        %%%%% END OF UTIL CODE %%%%%
    end
end