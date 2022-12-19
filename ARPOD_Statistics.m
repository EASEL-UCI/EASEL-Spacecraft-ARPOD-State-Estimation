classdef ARPOD_Statistics
    properties
        fuelConsumed = 0;
        currTraj = [0;0;0;0;0;0];
        
        trackTraj = [];
        trackEstTraj = [];
        trackU = [];
        trackFuelConsumption = [];
        trackPhase = [];
        timestamps = [];
        total_steps = 0;

        estimation_ts = [0];
    end
    methods
        function obj = initBenchmark(obj, traj0)
            obj.currTraj = traj0;
            obj.trackFuelConsumption = [0];
            obj.timestamps = [0];
            obj.trackPhase = [ARPOD_Benchmark.calculatePhase(traj0,0)]; %mission always starts on phase 1
            obj.trackTraj = [obj.trackTraj, traj0];
            obj.trackEstTraj = [obj.trackEstTraj, traj0];
            obj.total_steps = 1;
        end
        function obj = updateBenchmark(obj, u, mass, trueTraj, estTraj, tstep, estimation_time, phase)
            obj.trackU = [obj.trackU, u];

            fuel = obj.trackFuelConsumption(length(obj.trackFuelConsumption));
            % fuel -> newton seconds or kg*m/s or N*s
            fuel = fuel + sum(abs(u)) * mass * tstep; 
            obj.trackFuelConsumption = [obj.trackFuelConsumption, fuel];
            obj.trackPhase = [obj.trackPhase, phase];

            obj.currTraj = trueTraj;
            obj.trackEstTraj = [obj.trackEstTraj, estTraj];
            obj.trackTraj = [obj.trackTraj, obj.currTraj];
            obj.total_steps = obj.total_steps + 1;
            obj.timestamps = [obj.timestamps, obj.timestamps(length(obj.timestamps)) + tstep];

            obj.estimation_ts = [obj.estimation_ts, estimation_time];
        end
        function obj = graphLinear(obj, theta1, theta2)
            %{
                Graph Constituents
                ------------------
                    - Chaser Start
                    - Chaser End
                    - Chaser Trajectory
                    - Origin 'x' (target position)
                    - 4th rendezvous location 'x'
                    - LOS boundary (linear pyramid)
                    - Phase 2 boundary
                    - Phase 3 boundary
            %}

            set(groot,'defaultAxesTickLabelInterpreter','latex');  
            set(groot,'defaulttextinterpreter','latex');
            set(groot,'defaultLegendInterpreter','latex');
            
            fsize = 20; % figure fontsize
            lw = 2; % linewidth
            
            figure(1)
            %draw phase 2 sphere boundary
            [x,y,z] = sphere(10);
            r = ARPOD_Benchmark.rho_r;
            scatter3(r*x(:),r*y(:),r*z(:), 5,'o', 'filled', 'MarkerEdgeColor', 'c');
            axis equal
            hold on

            %draw phase 3 sphere boundary
            [x,y,z] = sphere(5);
            r = ARPOD_Benchmark.rho_d;
            scatter3(r*x(:),r*y(:),r*z(:), 5, 'o', 'filled', 'MarkerEdgeColor', 'm');

            c = ARPOD_Benchmark.rho_d;
            %drawing pillars of pyramid
            %plot3([0,-sin(theta2/2)*c], [0,-sin(theta1/2)*c], [0,-cos(theta1/2)*c], 'g');
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
            
            %draw truechaserTraj
            plot3(obj.trackTraj(1,:), obj.trackTraj(2,:), obj.trackTraj(3,:), 'r');
            %draw estchaserTraj
            plot3(obj.trackEstTraj(1,:), obj.trackEstTraj(2,:), obj.trackEstTraj(3,:), 'b');

            %draw target position
            %draw chaserStart
            %draw chaserEnd
            %draw 
            title("Chaser Trajectory")
            xlabel("x [km]")
            ylabel("y [km]")
            zlabel("z [km]")
            set(gca,'fontsize',fsize)
            set(gcf,'Position', [10 10 500 500])
            set(gca, 'TickLabelInterpreter', 'latex')
            hold off

            figure(2)
            set(gcf,'Position', [10 10 800 800])
            subplot(4,1,1)
            plot(obj.timestamps, obj.trackFuelConsumption, 'b','LineWidth',lw)
%             title("Time vs. Total Fuel Consumed")
            xlabel("Time [s]")
            ylabel("Fuel [kg m/s]") 
            set(gca,'fontsize',fsize)
            set(gca, 'TickLabelInterpreter', 'latex')
            grid
            subplot(4,1,2)
%             sq_error = sum((obj.trackTraj - obj.trackEstTraj).^2);
            mean_squared_error = (1/length(obj.trackTraj))*sum((obj.trackTraj - obj.trackEstTraj).^2);
%             plot(obj.timestamps, sq_error, 'r')
            plot(obj.timestamps, mean_squared_error, 'r','LineWidth',lw)
%             title("Squared Error of Trajectory Over Time")
            xlabel("Time [s]")
            ylabel("MSE of $\bf{x}$") % mean squared error of entire state estimate vector
            set(gca,'fontsize',fsize)
            set(gca, 'TickLabelInterpreter', 'latex')
            grid
            subplot(4,1,3)
%             sq_error = sum((obj.trackTraj - obj.trackEstTraj).^2);
            mean_squared_error_pos = (1/length(obj.trackTraj(1:3,:)))*sum((obj.trackTraj(1:3,:) - obj.trackEstTraj(1:3,:)).^2);
%             plot(obj.timestamps, sq_error, 'r')
            plot(obj.timestamps, mean_squared_error_pos, 'r','LineWidth',lw)
%             title("Squared Error of Trajectory Over Time")
            xlabel("Time [s]")
            ylabel("MSE of pos") % mean squared error of entire state estimate vector
            set(gca,'fontsize',fsize)
            set(gca, 'TickLabelInterpreter', 'latex')
            grid
            subplot(4,1,4)
%             sq_error = sum((obj.trackTraj - obj.trackEstTraj).^2);
            mean_squared_error_vel = (1/length(obj.trackTraj(4:6,:)))*sum((obj.trackTraj(4:6,:) - obj.trackEstTraj(4:6,:)).^2);
%             plot(obj.timestamps, sq_error, 'r')
            plot(obj.timestamps, mean_squared_error_vel, 'r','LineWidth',lw)
%             title("Squared Error of Trajectory Over Time")
            xlabel("Time [s]")
            ylabel("MSE of vel") % mean squared error of entire state estimate vector
            set(gca,'fontsize',fsize)
            set(gca, 'TickLabelInterpreter', 'latex')
            grid

            figure(3) % true trajectories and estimates in 2D planes
            set(gcf,'Position', [10 10 800 800])
            subplot(3,1,1) % x-y
            plot(obj.trackTraj(1,:), obj.trackTraj(2,:), 'r','LineWidth',lw)
            hold on
            plot(obj.trackEstTraj(1,:), obj.trackEstTraj(2,:), 'b--','LineWidth',lw)
%             title('Chaser Trajectory 2D')
            xlabel("$x$ [km]")
            ylabel("$y$ [km]")
            set(gca,'fontsize',fsize)
            legend('true','estimate','Location','southeast')
            set(gca, 'TickLabelInterpreter', 'latex')
            grid
            subplot(3,1,2) % x-z
            plot(obj.trackTraj(1,:), obj.trackTraj(3,:), 'r','LineWidth',lw)
            hold on
            plot(obj.trackEstTraj(1,:), obj.trackEstTraj(3,:), 'b--','LineWidth',lw)
%             title('Chaser Trajectory 2D')
            xlabel("$x$ [km]")
            ylabel("$z$ [km]")
            legend('true','estimate')
            set(gca,'fontsize',fsize)
            set(gca, 'TickLabelInterpreter', 'latex')
            grid
            subplot(3,1,3) % y-z
            plot(obj.trackTraj(2,:), obj.trackTraj(3,:), 'r','LineWidth',lw)
            hold on
            plot(obj.trackEstTraj(2,:), obj.trackEstTraj(3,:), 'b--','LineWidth',lw)
%             title('Chaser Trajectory 2D')
            xlabel("$y$ [km]")
            ylabel("$z$ [km]")
            legend('true','estimate')
            set(gca,'fontsize',fsize)
            set(gca, 'TickLabelInterpreter', 'latex')
            grid

            figure(4)
            plot(obj.timestamps, obj.estimation_ts, 'b');
            xlabel("Time (seconds)")
            ylabel("State Estimation Time (seconds)")
            return;
        end
        function totalMSE = getError(obj)
            MSEperStep = sum((obj.trackTraj - obj.trackEstTraj).^2);
            totalMSE = sum(MSEperStep);
        end
    end
end