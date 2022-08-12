classdef ChaserMHE
    properties 
        n_horizon
        window_measurements % these are observed in simulation
        window_states %these are windows of states
        window_stateError % windows of state errors in optimization
        window_measError % windows of measurement errors in optimization
        state

        %optional feature: forgetting factor. Almost opposite to a discount
        %factor.
    end
    methods (Static)
        %{
            Vector: [x0:N ... w0:N ... v0:N]
            NOTE: v changes according to the phase
        %}

        %setup cost functions
        function objective = quadraticCost()
            % cost function with relation to additive costs quadraticically
        end
        function objective = loglikelihoodCost()
        end

        %setup constraints 
        function setupEqualityConstraints()
        end
        function setupInequalityConstraints()
        end

        %convert trajectory into windows for MHE
        function vectorToWindows()
        end

    end
    methods 
        function initMHE(obj, traj0, n_horizon)
        end

        %window functions
        function vector = windowsToVector(obj)
        end
        function obj = windowShift(obj, meas)
        end

        %optimization functions
        function warmStart()
        end
        function optimize()
        end

        %estimate (piecing it together)
        function estimate() %ducktyping for all state estimators
        end        

    end
    methods (Static)
        function next_state = linearDynamics(x, u, R, T)
            mu_GM = 398600.4; %km^2/s^2;

            n = sqrt(mu_GM / (R.^3) );
            A = zeros(6,6);
            B = zeros(6,3);
            S = sin(n * T);
            C = cos(n * T);

            A(1,:) = [4-3*C,0,0,S/n,2*(1-C)/n,0];
            A(2,:) = [6*(S-n*T),1,0,-2*(1-C)/n,(4*S-3*n*T)/n,0];
            A(3,:) = [0,0,C,0,0,S/n];
            A(4,:) = [3*n*S,0,0,C,2*S,0];
            A(5,:) = [-6*n*(1-C),0,0,-2*S,4*C-3,0];
            A(6,:) = [0,0,-n*S,0,0,C];

            B(1,:) = [(1-C)/(n*n),(2*n*T-2*S)/(n*n),0];
            B(2,:) = [-(2*n*T-2*S)/(n*n),(4*(1-C)/(n*n))-(3*T*T/2),0];
            B(3,:) = [0,0,(1-C)/(n*n)];
            B(4,:) = [S/n,2*(1-C)/n, 0];
            B(5,:) = [-2*(1-C)/n,(4*S/n) - (3*T),0];
            B(6,:) = [0,0,S/n];

            next_state = A*x + B*u;
        end
        function states = optimize(measurements, u, state0, traj0, weightW, weightV, N, tstep, R, options)
            %{
                measuremnts shape: (3,N)
                tra j0 shape: (6,N)
                % for now time invariant
                % looks into forgetting factor (weigh higher newer terms)
                weightW shape: (6,6,N)
                weightV shape: (3,3,N)
                (x0 - xbar0)*weight0*(x0-xbar0) + wT P_w w + vT Pv v

                x0: true value
                [x0, x1, x2, x3] -> optimize x0->3
                [x1, x2, x3, x4] -> x1->x4
                Note: initialize w/ propagating thru dynamics
                Note: then initialize w/ previous mhe estimates
            %}
            [n_V, m_V, step_V] = sizes(weightV);
            meas_size = n_V;
            function cost = objective(x)
                %{
                    x: x consists of x_0:N, w_0:N, v_0:N (size is 6*N + 6*N
                    + 3*N, 1)
                %}
                wt = x(6*N+1:12*N,:);

                vt = x(12*N+1:(12+meas_size)*N,:);
                cost = 0;
                for i = 1:N
                    wi = wt(1+6*(i-1):6*i,:);
                    vi = vt(1+(meas_size)*(i-1):(meas_size)*i,:);
                    cost = cost + wi.' * weightW(:,:,i) * wi;
                    cost = cost + vi.' * weightV(:,:,i) * vi;
                end
                %cost returned
            end

            function [c,ceq] = nonlcon(x)
                c = 0; %no inequality constraints

                xt = x(1:6*N,:);
                wt = x(6*N+1:12*N,:);
                vt = x(12*N+1:(12+meas_size)*N,:);

                ceq = zeros(meas_size*N+6*N+6,1);
                for i = 1:N
                    xi = xt(6*(i-1)+1:6*i,:);
                    if i < N
                        xi1 = xt(6*i+1:6*(i+1),:);
                        wi = wt(6*(i-1)+1:6*i,:);
                        ceq(meas_size*N+6*(i-1)+1:meas_size*N+6*i,:) = ChaserMHE.linearDynamics(xi,u(:,i), R, tstep) + wi - xi1;
                    end
                    hi = ARPOD_Sensing.measure(xi);
                    vi = vt(1+meas_size*(i-1):meas_size*i,:);
                    ceq(meas_size*(i-1)+1:meas_size*i,:) = hi + vi - measurements(:,i);
                end
                ceq(meas_size*N+6*N+1:meas_size*N+6*N+6,:) = x(1:6,:) - traj0;
            end
            %initialize guess (warm starting?)
            x0 = zeros(15*N,1);
            [n_dim, n_horizon] = size(state0);
            x0(1:n_dim*n_horizon,:) = reshape(state0,n_dim*n_horizon,1);
            A = [];
            b = [];
            Aeq = [];
            beq = [];
            lb = zeros((12+meas_size)*N,1) - Inf; %have bounds for noise
            ub = zeros((12+meas_size)*N,1) + Inf; %
            nonlincon = @nonlcon;
            xstar = fmincon(@objective, x0, A, b, Aeq, beq, lb, ub, nonlincon, options);
            xstar_xt = xstar(1:6*N,:);

            states = reshape(xstar_xt,6,N); 
        end
    end
end
