classdef ChaserMHE_Unconstr
    properties 
        n_horizon
        window_measurements = []; % these are observed in simulation
        window_states %these are windows of states
        window_control = [];
        state

    end
    methods (Static)  
        function modified_sense = senseModify(measurement)
            [dim,i] = size(measurement);
            modified_sense = measurement;
            if dim == 2
                modified_sense(3) = 0;
            end
        end
    end
    methods
        function obj = init(obj, traj0, meas, n_horizon, tstep)
            obj.n_horizon = n_horizon;

            [A,B] = ARPOD_Benchmark.linearDynamics(tstep);
            window_state = zeros(6,n_horizon);
            window_state(:,1) = traj0;
            for i = 2:n_horizon
                window_state(:,i) = A*window_state(:,i-1);
            end
            obj.window_states = window_state;
            obj.window_measurements = ChaserMHE.senseModify(meas);
        end

        function vector = windowsToVector(obj, n_horizon)
            vector = [];
            for i = 1:n_horizon
                vector = [vector; obj.window_states(:,i)];
            end
        end
        function obj = vectorToWindows(obj, vector, n_horizon)
            for i = 1:n_horizon
                obj.window_states(:,i) = vector(6*(i-1)+1:6*(i-1)+6);
            end
        end
        function obj = windowShift(obj, meas, control, tstep)
            [A,B] = ARPOD_Benchmark.linearDynamics(tstep);
            [dim, horizon] = size(obj.window_measurements);


            if horizon < obj.n_horizon
                obj.window_measurements = [obj.window_measurements, ChaserMHE_Unconstr.senseModify(meas)];
                obj.window_control = [obj.window_control, control];
                obj.window_states(:,horizon+1) = A*obj.window_states(:,horizon) + B*control;

            else
                obj.window_measurements = [obj.window_measurements(:,2:obj.n_horizon), ChaserMHE.senseModify(meas)];
                obj.window_states = [obj.window_states(:,2:obj.n_horizon), A*obj.window_states(:,obj.n_horizon) + B*control];

                [dim, n] = size(obj.window_control);
                obj.window_control = [obj.window_control(:,2:n), control];
            end
        end

        function cost = quadraticCost(obj, Q, R, n_horizon, tstep)
            [A,B] = ARPOD_Benchmark.linearDynamics(tstep);
            function e = Cost(x)
                e = 0;
                for i = 1:n_horizon
                    if i ~= n_horizon
                        traj = x(6*(i-1)+1:6*(i-1)+6);
                        next_traj = x(6*(i)+1:6*(i)+6);
                        term = next_traj - (A*traj + B*obj.window_control(:,i));
                        e = e + term.'*Q*term;
                    end
                    traj = x(6*(i-1)+1:6*(i-1)+6);
                    traj_meas = ARPOD_Sensing.measure(traj);
                    term = obj.window_measurements(:,i) - traj_meas;
                    if obj.window_measurements(3,i) == 0
                        term(3) = 0;
                    end
                    e = e + term.'*R*term;
                end
            end
            cost = @Cost;
        end
        function obj = optimize(obj, Q_cov, R_cov, tstep)
            [dim, horizon] = size(obj.window_measurements);

            objective = obj.quadraticCost(Q_cov,R_cov,horizon,tstep);

            x0 = obj.windowsToVector(horizon);

            xstar = fminunc(objective, x0);
            %
            [dim, horizon] = size(xstar);
            if horizon == 0
                error("Optimization went wrong!!");
            end
            disp(objective(xstar))
            obj = obj.vectorToWindows(xstar, horizon);
        end
        function obj = estimate(obj, control, measurement, Q_cov, R_cov,tstep, phase) %ducktyping for all state estimators
            obj = obj.windowShift(measurement, control, tstep);
            obj = obj.optimize(Q_cov, R_cov, tstep);

            obj.state = obj.window_states(:,length(obj.window_states));
        end     
    end
end