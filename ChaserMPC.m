classdef ChaserMPC
    methods (Static)
        function [A,B] = linearDynamics(T)
            mu_GM = 398600.4; %km^2/s^2;
            R = ARPOD_Benchmark.a;

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

        end
        function [xs,us] = optimizeLinear(traj0, traj_horizon, Q, R, n_horizon, timestep, phase)
            %{
                min x^T*Q*x + u^T*R*u
                    s.t. x_k+1 = Ax_k + Bu_k
                         -umax <= u <= umax
                    Phase 3 Constraints:
                         %Line of Sight constraint for docking phase 3
                         LOS*x <= 0

                         %infinity norm version of velocity constraints
                         [xdot,ydot,zdot] <= Vbar
                         -[xdot,ydot,zdot[ <= Vbar 
            %}
            [A,B] = ChaserMPC.linearDynamics(timestep);

            %setting up H matrix (combined Q and R)
            H = [];
            for i = 1:n_horizon
                H = [H, diag(Q).'];
                if i ~= n_horizon
                    H = [H, diag(R).'];
                end
            end
            ndim = length(H);
            f = zeros(ndim,1);

            H = sparse(diag(H));

            % H represents the condensed quadratic version of 
            % sum_1->N x^T*Q*x + u^T*R*u

            %constructing equality matrix
            Aeq = [];
            beq = zeros(9*n_horizon-3,1);
            for i = 1:n_horizon-1

                row = [];
                for j = 1:n_horizon
                    if j == i
                        row = [row, A,B];
                    elseif j == i+1
                        row = [row, -eye(6), zeros(6,3)];
                    elseif j == n_horizon
                        row = [row, zeros(6,6)];
                    else
                        row = [row, zeros(6,6), zeros(6,3)];
                    end
                end

                Aeq = [Aeq;row];
            end
            
            %last iteration, n_horizon for the last equality constraint,
            %x0 = xbar0
            row = [];
            for i = 1:n_horizon-1
                if i == 1
                    row = [row, eye(6), zeros(6,3)];
                else
                    row = [row, zeros(6,6), zeros(6,3)];
                end
            end
            Aeq = [Aeq;row];
            beq(9*n_horizon-5:9*n_horizon) = traj0;

            %setting up bound constraints
            ubar = ARPOD_Benchmark.ubar;
            ub = [];
            lb = [];
            for i = 1:n_horizon-1
                ub = [ub; inf;inf;inf;inf;inf;inf];
                ub = [ub; ubar;ubar;ubar];
            end
            ub = [ub; inf;inf;inf;inf;inf;inf];
            lb = -ub;

            % ig it's phase 3, add in the LOS constraint and vbar
            % constraint
            vbar = ARPOD_Benchmark.Vbar;
            A = [];
            b = zeros(4*n_horizon);
            if phase == 3
                %LOS Matrix
                theta1 = ARPOD_Benchmark.theta * pi / 180;
                theta2 = theta1;
                LOS = [ sin(theta1/2), cos(theta1/2), 0; 
                        sin(theta1/2), -cos(theta1/2), 0;
                        sin(theta2/2), 0, cos(theta2/2);
                        sin(theta2/2), 0, -cos(theta2/2)];
                LOS = [LOS, zeros(4,3)];

                for i = 1:9:9*n_horizon-3
                    %vbar constraints
                    ub(i+3:i+5) = [vbar;vbar;vbar];
                    lb(i+3:i+5) = [-vbar;-vbar;-vbar];
                    
                    A = [A; LOS];
                end
            end


            % in case this doesn't work, add in the traj_horizon as the
            % initial input.
            x = quadprog(H,f,A,b,Aeq,beq,lb,ub, traj_horizon);

            xs = [];
            us = [];
            for i = 1:n_horizon
                x_id = 9*(i-1)+1;
                xs = [xs, x(x_id:x_id+5)];
                if i ~= n_horizon
                    u_id = 9*(i-1)+1+6;
                    us = [us, x(u_id:u_id+2)];
                end
            end
            % return xs and us
        end
        function us = optimizeNonlinear()
        end
        function u = mpc_estimate()
        end
    end
end