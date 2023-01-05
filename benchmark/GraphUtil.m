
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

        $ see if it looks nice
        - EKF,PF, MHE Starting and Ending Points Distribution
        
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

    - REGISTER FOR CONFERENCE!!!
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

            [x,y,z] = sphere(5);
            r = ARPOD_Benchmark.rho_d;
            scatter3(r*x(:),r*y(:),r*z(:), 5, 'o', 'filled', 'MarkerEdgeColor', 'm');

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
                axis equal
                hold on
            end
        end
        function void = graphTrajectory(x,y,z,color,first_graph)
            plot3(x,y,z,color);
            if first_graph
                axis equal
                hold on
            end
        end
        function void = graphTranslation(x,y,z,first_graph)
            t = length(x);
            ts = linspace(0,t,t);
            plot(ts,x,'r');
            if first_graph
                axis equal
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
                    axis equal
                    hold on 
                    first_graph = false;
                end
            end
            GraphUtil.graphTarget();
            hold off
        end

        function percentages = normalizeTrajectories(track_trajs)
            num_steps = length(track_trajs);
            distance = 0;
            percentages = zeros(num_steps);
            for idx = 2:num_steps
                distance = distance + sqrt(sum((track_trajs(idx) - track_trajs(idx-1)).^2));
                percentages(idx) = distance;
            end
            percentages = percentages / distance;
        end
        function void = graphTrajectorySpecial(arpod_stats)
            first_graph = true;
            sum_trajs = 0;
            for idx = 1:length(arpod_stats)
                track_trajs = arpod_stats{idx}.trackTraj;
                %normalize trajectories (get x axis stuff)
                percent_trajs = GraphUtil.normalizeTrajectories(track_trajs);

                %get y axis stuff
                arpod_stats{idx}.
            end

            %print x and y axis points as an error bar w/ stdev....
            
        end
        %%%%% END OF UTIL CODE %%%%%
    end
end