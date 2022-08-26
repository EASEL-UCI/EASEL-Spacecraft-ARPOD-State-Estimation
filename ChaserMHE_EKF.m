classdef ChaserMHE_EKF
    %{
        min ||v||_Q^2 + ||w||_R^2
        s.t. x_K+1 = Ax_k + Bu_k
             y_k = x_k + w_k

        QP
    %}
    properties
        ekf

        n_horizon
        window_measurements = []; % these are observed in simulation
        window_states %these are windows of states
        window_stateError % windows of state errors in optimization
        window_measError % windows of measurement errors in optimization
        window_control = [];
        state

        meas

        forget_factor;

        ekfQ
        ekfR
    end
    methods (Static)
        function [xt,wt,vt] = vectorIterationHelper(vector, horizon)
            N = horizon;
            xt = vector(1:6*N,:);
            wt = vector(6*N+1:12*N,:);
            vt = vector(12*N+1:18*N,:);
        end

        function [xi,wi,vi] = vectorIndexHelper(xt,wt,vt, i)
            xi = xt(6*(i-1)+1:6*i,:);
            wi = wt(6*(i-1)+1:6*i,:);
            vi = vt(6*(i-1)+1:6*i,:);
        end

        function [H,f] = quadraticCost(horizon, Q, R)
            % cost function with relation to additive costs quadraticically
            % Q is the weighting matrix for state errors
            % R is weighting matrix for measurement errors
            H = [];
            for i = 1:horizon
                H = [H; zeros(6,6*3*horizon)];
            end
            for i = 1:horizon
                Hrow = [];
                for j = 1:3*horizon
                    if j == horizon+i
                        Hrow = [Hrow, Q];
                    else
                        Hrow = [Hrow, zeros(6,6)];    
                    end
                end
                H = [H;Hrow];
            end
            for i = 1:horizon
                Hrow = [];
                for j = 1:3*horizon
                    if j == 2*horizon+i
                        Hrow = [Hrow, R];
                    else
                        Hrow = [Hrow, zeros(6,6)];    
                    end
                end
                H = [H;Hrow];
            end
            f = zeros(length(H),1);
        end
    end
    methods 
        function obj = init(obj, traj0, n_horizon, forget_factor, tstep, ekfQ, ekfR)
            obj.n_horizon = n_horizon;
            window_state = zeros(6,n_horizon);

            [A,B] = ARPOD_Benchmark.linearDynamics(tstep);
            window_state(:,1) = traj0;
            for i = 2:n_horizon
                window_state(:,i) = A*window_state(:,i-1);
            end
            obj.window_measError = 100*ones(6,n_horizon);
            obj.window_stateError = 100*ones(6,n_horizon);
            obj.window_states = window_state;
            obj.forget_factor = forget_factor;

            obj.window_measurements = traj0;

            obj.ekf = ChaserEKF;
            obj.ekf = obj.ekf.initEKF(traj0, 1e-20*eye(6));
            obj.ekfQ = ekfQ;
            obj.ekfR = ekfR;
        end

        function obj = fixEKF(obj, seR)
            obj.ekf.state = obj.state;
            obj.ekf.cov = seR.^(-1);
        end
        function obj = measureEKF(obj, control, meas, tstep, phase)
            obj.ekf = obj.ekf.estimate(control, meas, obj.ekfQ, obj.ekfR, tstep, phase);
            obj.meas = obj.ekf.state;
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

            [xt,wt,vt] = ChaserMHE_EKF.vectorIterationHelper(vector, horizon);
            for i = 1:horizon
                [xi,wi,vi] = ChaserMHE_EKF.vectorIndexHelper(xt,wt,vt,i);
                obj.window_states(:,i) = xi;
                obj.window_stateError(:,i) = wi;
                obj.window_measError(:,i) = vi;
            end
            %return obj
        end

        function obj = windowShift(obj, meas, control, tstep)
            %meas is now sizes 6
            [A,B] = ARPOD_Benchmark.linearDynamics(tstep);
            [dim,num] = size(obj.window_measurements);

            if num < obj.n_horizon
                obj.window_measurements = [obj.window_measurements, meas];
                obj.window_control = [obj.window_control, control];

                obj.window_states(:,num+1) = A*obj.window_states(:,num) + B*control;
                obj.window_measError(:,num+1) = meas - obj.window_states(:,num+1);
            else

                obj.window_measurements = [obj.window_measurements(:,2:obj.n_horizon), meas];
                obj.window_states = [obj.window_states(:,2:obj.n_horizon), A*obj.window_states(:,obj.n_horizon) + B*control];
                obj.window_stateError = [obj.window_stateError(:,2:obj.n_horizon), 100*ones(6,1)];

                obj.window_measError(:,num+1) = meas - obj.window_states(:,obj.n_horizon);

                [dim, n] = size(obj.window_control);
                obj.window_control = [obj.window_control(:,2:n), control];
            end
        end

        function [Aeq, beq] = setupDynamicConstraints(obj, horizon, tstep)
            Aeq = [];
            beq = [];
            [A,B] = ARPOD_Benchmark.linearDynamics(tstep);
            for i = 1:horizon-1
                beq = [beq; -B*obj.window_control(:,i)];

                Arow = [];
                for j = 1:3*horizon
                    if j == i
                        Arow = [Arow, A];
                    elseif j == i+1
                        Arow = [Arow, -eye(6)];
                    elseif j == horizon+i
                        Arow = [Arow, eye(6)];
                    else
                        Arow = [Arow, zeros(6,6)];
                    end
                end
                Aeq = [Aeq; Arow];
            end
        end
        function [Aeq, beq] = setupMeasurementConstraints(obj, horizon)
            Aeq = [];
            beq = [];
            for i = 1:horizon
                beq = [beq; obj.window_measurements(:,i)];

                Arow = [];
                for j = 1:3*horizon
                    if j == i
                        Arow = [Arow, eye(6)];
                    elseif j == 2*horizon+i
                        Arow = [Arow, eye(6)];
                    else
                        Arow = [Arow, zeros(6,6)];
                    end
                end
                Aeq = [Aeq; Arow];
            end
        end

        function obj = optimize(obj, Q_cov, R_cov, tstep)
            [dim, horizon] = size(obj.window_measurements);

            [H,f] = ChaserMHE_EKF.quadraticCost(horizon, Q_cov, R_cov);

            x0 = obj.windowsToVector();
            [Aeq1,beq1] = obj.setupDynamicConstraints(horizon, tstep);
            [Aeq2, beq2] = obj.setupMeasurementConstraints(horizon);
            Aeq = [Aeq1;Aeq2];
            beq = [beq1;beq2];
            lb = ones(length(x0),1) - Inf;
            ub = ones(length(x0),1) + Inf;

            disp(size(Aeq1))
            disp(size(Aeq2))
            disp(size(H))
            %options = optimoptions(@quadprog, 'Algorithm', 'trust-region-reflective');
            xstar = quadprog(H,f,[],[],Aeq,beq,lb,ub,x0);

            %
            [dim, horizon] = size(xstar);
            if horizon == 0
                error("Optimization went wrong!!");
            end
            disp(size(xstar))
            disp(xstar.' * H * xstar)
            obj = obj.vectorToWindows(xstar);
        end
        %estimate (piecing it together)
        function obj = estimate(obj, control, measurement, Q_cov, R_cov,tstep, phase) %ducktyping for all state estimators
            obj = obj.measureEKF(control, measurement,tstep, phase);
            

            obj = obj.windowShift(obj.meas,control,tstep);
            obj = obj.optimize(Q_cov,R_cov,tstep);

            obj.state = obj.window_states(:,length(obj.window_states));
            obj.fixEKF(R_cov.^(-1));
        end  
    end
end