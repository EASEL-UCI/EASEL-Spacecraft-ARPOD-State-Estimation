classdef ChaserNLMPC
    %{
        Re-uses a lot of software from ChaserMPC
        Nonlinear MPC file is for all exclusive nonlinear

        This code will differ from the linear MPC not just in the fact that
        it is linear, but in the fact that since there is a need for warm
        starting, the code structure will be a bit different.

        Ideally, the warm-starting stays inside this script file, so the
        user will not have to worry about the details.
    %}
    properties
        warm_traj = [0;0;0;0;0;0];
        warm_u = [0;0;0];
        n_horizon
    end
    methods (Static)
        function c = coneConstraint(xs,n_horizon)
            c = zeros(n_horizon,1);
            c_vec = ARPOD_Benchmark.rho_d * ARPOD_Benchmark.c;
            for i = 1:n_horizon
                pos = xs(1:3,i);
                c(i,:) = cos(ARPOD_Benchmark.theta/2) - pos.' * c_vec / (norm(pos)*norm(c_vec));
            end
        end
        function c = vbarConstraint(xs, n_horizon)
            c = zeros(n_horizon,1);
            for i = 1:n_horizon
                v = xs(3:6,i);
                c(i,:) = norm(v) - ARPOD_Benchmark.Vbar;
            end
        end
        function costF = quadraticCost(Q, R, n_horizon)
            %{
                Parameters:
                -----------
                Description:
                ------------
                    Sets up the quadratic cost function specifically for
                    fmincon. Hence why it is needed to make a difference.
                    Ideally, can switch off different cost functions.
            %}
            [H,f] = ChaserMPC.setupQuadraticCost(Q,R,n_horizon);
            costF = @(x) x.'*H*x;
        end
    end
    methods
        function obj = initMPC(obj, traj0, n_horizon, tstep)
            obj.n_horizon = n_horizon;
            obj.warm_traj = zeros(6,n_horizon);
            obj.warm_traj(:,1) = traj0;
            obj.warm_u = zeros(3,n_horizon-1);

            traj = traj0;
            [A,B] = ARPOD_Benchmark.linearDynamics(tstep);
            for i = 2:n_horizon
                traj = A*traj;
                obj.warm_traj(:,i) = traj;
            end
            % warm start is now the predicted Dynamics.
        end
        function obj = getWarmStart(obj, traj0, tstep)
            [A,B] = ARPOD_Benchmark.linearDynamics(tstep);
            next_traj = A * obj.warm_traj(:,obj.n_horizon);
            obj.warm_traj = [traj0, obj.warm_traj(:,3:obj.n_horizon), next_traj];
            obj.warm_u = [obj.warm_u(:,2:obj.n_horizon-1), obj.warm_u(:,obj.n_horizon-1)];
        end
        function fullVector = convertWarmTraj(obj)
            fullVector = [];
            for i = 1:obj.n_horizon
                fullVector = [fullVector; obj.warm_traj(:,i)];
                if i ~= obj.n_horizon
                    fullVector = [fullVector; obj.warm_u(:,i)];
                end
            end
            %return fullVector
        end
        function nonlcon = setupNonLCon(obj, phase)
            function [c,ceq] = nonlconf(x)
                ceq = [];
                if phase == 1
                    c = [];
                elseif phase == 2
                    [xs,us] = ChaserMPC.extractOptVector(x, obj.n_horizon);
                    c = ChaserNLMPC.coneConstraint(xs, obj.n_horizon);
                    c = c(length(c),:); %only get the last trajectory LOS
                else %phase == 3
                    [xs,us] = ChaserMPC.extractOptVector(x, obj.n_horizon);
                    c = ChaserNLMPC.coneConstraint(xs, obj.n_horizon);
                    c = [c;ChaserNLMPC.vbarConstraint(xs,obj.n_horizon)];
                end
                %setup all of the constraints necessary
            end
            nonlcon = @nonlconf;
        end
        function [xs,us] = optimize(obj, traj0, Q, R, tstep, mass, phase)
            %{
                
                Assumes is already has a warm start and n_horizon.
            %}
            %setup objective function
            fun = ChaserNLMPC.quadraticCost(Q,R,obj.n_horizon);

            %setup dynamic equality constraints
            [Aeq, beq] = ChaserMPC.setupLinearConstraints(traj0, tstep, obj.n_horizon);

            %setting up bounded ubar constraints
            [ub,lb] = ChaserMPC.setupControlInputBoundaries(obj.n_horizon, mass);

            %setup inequality constraints that are nonlinear
            nonlcon = obj.setupNonLCon(phase);

            %get warm start initial solution in vector form
            vectorWarmStart = obj.convertWarmTraj();

            options = optimoptions(@fmincon, 'Algorithm', 'sqp', 'MaxIterations', 2000, 'ConstraintTolerance', 1e-6);
            x = fmincon(fun,vectorWarmStart,[],[],Aeq,beq,lb,ub,nonlcon, options);
            [xs,us] = ChaserMPC.extractOptVector(x, obj.n_horizon);

        end
        function obj = controlMPC(obj, traj0, Q, R, tstep, n_horizon, mass, phase)
            [dim, n] = size(obj.warm_traj);
            if n ~= n_horizon
                obj = obj.initMPC(traj0, n_horizon, tstep);
            else
                obj = obj.getWarmStart(traj0, tstep);
            end
            [xs,us] = obj.optimize(traj0, Q, R, tstep, mass, phase);
            obj.warm_traj = xs;
            obj.warm_u = us;
        end
    end
end