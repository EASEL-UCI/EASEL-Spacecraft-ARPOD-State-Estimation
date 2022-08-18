classdef ChaserMHE
    properties 
        n_horizon
        window_measurements = []; % these are observed in simulation
        window_states %these are windows of states
        window_stateError % windows of state errors in optimization
        window_measError % windows of measurement errors in optimization
        window_control = [];
        state

        forget_factor;
    end
    methods (Static)
        %{
            Vector: [x0:N ... w0:N ... v0:N]
            NOTE: v changes according to the phase
        %}
        function [xt,wt,vt] = vectorIterationHelper(vector, horizon)
            N = horizon;
            xt = vector(1:6*N,:);
            wt = vector(6*N+1:12*N,:);
            vt = vector(12*N+1:15*N,:);
        end

        function [xi,wi,vi] = vectorIndexHelper(xt,wt,vt, i)
            xi = xt(6*(i-1)+1:6*i,:);
            wi = wt(6*(i-1)+1:6*i,:);
            vi = vt(3*(i-1)+1:3*i,:);
        end

        function modified_sense = senseModify(measurement)
            [dim,i] = size(measurement);
            modified_sense = measurement;
            if dim == 2
                modified_sense(3) = 0;
            end
        end

        %setup cost functions
        function objective = quadraticCost(horizon, Q, R, f_factor)
            % cost function with relation to additive costs quadraticically
            % Q is the weighting matrix for state errors
            % R is weighting matrix for measurement errors
            function error = quadcost(x)
                error = 0;
                [xt,wt,vt] = ChaserMHE.vectorIterationHelper(x,horizon);
                for i = 1:horizon
                    [xi,wi,vi] = ChaserMHE.vectorIndexHelper(xt,wt,vt,i);
                    stateE = wi;
                    errorState = stateE.' * Q * stateE;

                    measE = vi;
                    errorMeas = measE.' * R * measE;

                    error = error + f_factor.^(horizon-i) * (errorState + errorMeas);
                end
            end
            objective = @quadcost;
        end

        function objective = loglikelihoodCost(horizon, Q, R, f_factor)
            %
            %
            %
            function error = log_quadcost(x)
                error = 0;

                [xt,wt,vt] = ChaserMHE.vectorIterationHelper(x,horizon);
                for i = 1:horizon
                    [xi,wi,vi] = ChaserMHE.vectorIndexHelper(xt,wt,vt,i);
                    stateE = wi;
                    errorState = stateE.' * Q * stateE;

                    measE = vi;
                    errorMeas = measE.' * R * measE;

                    error = error + f_factor.^(horizon-i) * (errorState + errorMeas);
                end
                error = log(error);
            end
            objective = @log_quadcost;
        end

    end
    methods 
        function obj = initMHE(obj, traj0, meas, n_horizon, forget_factor, tstep)
            obj.n_horizon = n_horizon;
            window_state = zeros(6,n_horizon);

            [A,B] = ARPOD_Benchmark.linearDynamics(tstep);
            window_state(:,1) = traj0;
            for i = 2:n_horizon
                window_state(:,i) = A*window_state(:,i-1);
            end
            obj.window_measError = 100*ones(3,n_horizon);
            obj.window_stateError = zeros(6,n_horizon);
            obj.window_states = window_state;
            obj.forget_factor = forget_factor;

            obj.window_measurements = ChaserMHE.senseModify(meas);
        end

        %window functions
        function vector = windowsToVector(obj)
            [dim, horizon] = size(obj.window_measurements);
            vector = [];
            for i = 1:horizon
                vector = [vector; obj.window_states(:,i)];
            end
            for i = 1:horizon
                vector = [vector; obj.window_stateError(:,i)];
            end
            for i = 1:horizon
                vector = [vector; obj.window_measError(:,i)];
            end
            %vector is now [ all states, all state errors, all measurement
            %errors]
        end

        function obj = vectorToWindows(obj, vector)
            %go from vector to windows for object
            [dim, horizon] = size(obj.window_measurements);

            [xt,wt,vt] = ChaserMHE.vectorIterationHelper(vector, horizon);
            for i = 1:horizon
                [xi,wi,vi] = ChaserMHE.vectorIndexHelper(xt,wt,vt,i);
                obj.window_states(:,i) = xi;
                obj.window_stateError(:,i) = wi;
                obj.window_measError(:,i) = vi;
            end
            %return obj
        end

        function obj = windowShift(obj, meas, control, tstep)
            [A,B] = ARPOD_Benchmark.linearDynamics(tstep);
            [dim,num] = size(obj.window_measurements);

            if num < obj.n_horizon
                obj.window_measurements = [obj.window_measurements, ChaserMHE.senseModify(meas)];
                obj.window_control = [obj.window_control, control];

                obj.window_states(:,num+1) = A*obj.window_states(:,num) + B*control;
                state_meas = ARPOD_Benchmark.sensor(obj.window_states(:,num+1),@() [0;0;0], 2);
                obj.window_measError(:,num+1) = ChaserMHE.senseModify(meas) - state_meas;
            else

                obj.window_measurements = [obj.window_measurements(:,2:obj.n_horizon), ChaserMHE.senseModify(meas)];
                obj.window_states = [obj.window_states(:,2:obj.n_horizon), A*obj.window_states(:,obj.n_horizon) + B*control];
                obj.window_stateError = [obj.window_stateError(:,2:obj.n_horizon), zeros(6,1)];

                state_meas = ARPOD_Benchmark.sensor(obj.window_states(:,obj.n_horizon),@() [0;0;0], 2);
                error = ChaserMHE.senseModify(meas) - state_meas;
                obj.window_measError = [obj.window_measError(:,2:obj.n_horizon), error];

                [dim, n] = size(obj.window_control);
                obj.window_control = [obj.window_control(:,2:n), control];
            end
        end


        function ceq = setupDynamicConstraints(obj, vector, horizon, tstep)
            ceq = [];
            ceq = vector(1:6,:) - obj.window_states(:,1); % set x0 - xbar0 = 0;
            [A,B] = ARPOD_Benchmark.linearDynamics(tstep);


            [xt,wt,vt] = ChaserMHE.vectorIterationHelper(vector, horizon);
            for i = 1:horizon-1
                [xi,wi,vi] = ChaserMHE.vectorIndexHelper(xt,wt,vt,i);
                control = obj.window_control(:,i);

                state_ = xi;

                stateE_ = wi;

                %an offcase for the vectorIndexHelper
                next_state_ = xt(6*(i)+1:6*(i+1),:);

                ceq = [ceq; A*state_ + B*control + stateE_ - next_state_];  % Ax + Bu + vk - x_k+1 = 0
            end
        end
        function ceq = setupMeasurementConstraints(obj, vector, horizon)
            ceq = [];
            [xt,wt,vt] = ChaserMHE.vectorIterationHelper(vector, horizon);
            for i = 1:horizon

                [xi,wi,vi] = ChaserMHE.vectorIndexHelper(xt,wt,vt,i);
                state_ = xi;

                meas = obj.window_measurements(:,i);
                measE_ = vi;

                state_meas = ARPOD_Benchmark.sensor(state_, @() zeros(3,1), 2);
                if meas(3) == 0
                    state_meas(3) = 0;
                end
                ceq = [ceq; state_meas + measE_ - meas]; % g(x_k) + w_k - y_k = 0
            end
        end
        function nonlcon = setupEqualityConstraints(obj, horizon, tstep)
            function [c,ceq] = constrF(x)
                c = 0;
                ceq = obj.setupDynamicConstraints(x,horizon, tstep);
                ceq = [ceq; obj.setupMeasurementConstraints(x,horizon);];
            end
            nonlcon = @constrF;
        end

        %optimization functions
        function obj = optimize(obj, Q_cov, R_cov, tstep)
            [dim, horizon] = size(obj.window_measurements);

            nonlcon = obj.setupEqualityConstraints(horizon, tstep);
            objective = ChaserMHE.quadraticCost(horizon, Q_cov, R_cov, obj.forget_factor);

            x0 = obj.windowsToVector();
            A = [];
            b = [];
            Aeq = [];
            beq = [];
            lb = ones(length(x0),1) - Inf;
            ub = ones(length(x0),1) + Inf;

            options = optimoptions(@fmincon, 'Algorithm', 'sqp', 'MaxIterations', 2000, 'ConstraintTolerance', 1e-12);
            xstar = fmincon(objective, x0, A, b, Aeq, beq, lb, ub, nonlcon, options);
            %
            [dim, horizon] = size(xstar);
            if horizon == 0
                error("Optimization went wrong!!");
            end
            disp(objective(xstar))
            obj = obj.vectorToWindows(xstar);
        end

        %estimate (piecing it together)
        function obj = estimate(obj, control, measurement, Q_cov, R_cov,tstep, phase) %ducktyping for all state estimators
            obj = obj.windowShift(measurement,control,tstep);
            obj = obj.optimize(Q_cov,R_cov,tstep);

            obj.state = obj.window_states(:,length(obj.window_states));
        end        

    end
end
