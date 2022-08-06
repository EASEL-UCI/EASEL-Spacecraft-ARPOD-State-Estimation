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
    end
    methods
        function obj = initBenchmark(obj, traj0)
            obj.currTraj = traj0;
            obj.trackFuelConsumption = [0];
            obj.timestamps = [0];
            obj.trackPhase = [1]; %mission always starts on phase 1
            obj.total_steps = 1;
        end
        function obj = updateBenchmark(obj, u, mass, trueTraj, estTraj, tstep, phase)
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

            figure 

            [x,y,z] = sphere(10);
            r = ARPOD_Benchmark.rho_r;
            scatter3(r*x(:),r*y(:),r*z(:), 5,'o', 'filled', 'MarkerEdgeColor', 'c');

            axis equal
            hold on

            [x,y,z] = sphere(5);
            r = ARPOD_Benchmark.rho_d;
            scatter3(r*x(:),r*y(:),r*z(:), 5, 'o', 'filled', 'MarkerEdgeColor', 'm');

            c = ARPOD_Benchmark.rho_d;
            %drawing pillars of pyramid
            plot3([0,-sin(theta2/2)*c], [0,-sin(theta1/2)*c], [0,-cos(theta1/2)*c], 'g');
            plot3([0,-sin(theta2/2)*c], [0,sin(theta1/2)*c], [0,-cos(theta1/2)*c], 'g');
            plot3([0,sin(theta2/2)*c], [0,-sin(theta1/2)*c], [0,-cos(theta1/2)*c], 'g');
            plot3([0,sin(theta2/2)*c], [0,sin(theta1/2)*c], [0,-cos(theta1/2)*c], 'g');
            %drawing base of pyramid
            plot3([-sin(theta2/2)*c, sin(theta2/2)*c], [-sin(theta1/2)*c,-sin(theta1/2)*c], [-cos(theta1/2)*c,-cos(theta1/2)*c], 'g');
            plot3([-sin(theta2/2)*c, sin(theta2/2)*c], [sin(theta1/2)*c, sin(theta1/2)*c], [-cos(theta1/2)*c,-cos(theta1/2)*c], 'g');
            plot3([-sin(theta2/2)*c, -sin(theta2/2)*c], [-sin(theta1/2)*c,sin(theta1/2)*c], [-cos(theta1/2)*c,-cos(theta1/2)*c], 'g');
            plot3([sin(theta2/2)*c, sin(theta2/2)*c], [-sin(theta1/2)*c, sin(theta1/2)*c], [-cos(theta1/2)*c,-cos(theta1/2)*c], 'g');

            hold off

            return;
        end
    end
end