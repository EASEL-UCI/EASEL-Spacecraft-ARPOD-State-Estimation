classdef ChaserMPC
    methods (Static)
        function us = optimizeLinear(traj0, traj_horizon, Q, R, n_horizon, phase)
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
            % n_horizon is for the states and n_horizon-1 is for control inputs.
            names = cell(1,9*n_horizon-3);
            
            %modelling the objective function
            quadratic_Q = zeros(9*n_horizon-3);
            for i = 1:6:6*n_horizon
                %add all names to gurobi
                names(i) = {"x"+i};
                names(i+1) = {"y"+i};
                names(i+2) = {"z"+i};
                names(i+3) = {"xdot"+i};
                names(i+4) = {"ydot"+i};
                names(i+5) = {"zdot"+i};

                quadratic_Q(i) = Q(1,1);
                quadratic_Q(i+2) = Q(2,2);
                quadratic_Q(i+3) = Q(3,3);
                quadratic_Q(i+4) = Q(4,4);
                quadratic_Q(i+5) = Q(5,5);
                quadratic_Q(i+6) = Q(6,6);
            end
            for i = 6*n_horizon+1:3:9*n_horizon-3
                names(i) = {"ux"+i};
                names(i+1) = {"uy"+i};
                names(i+2) = {"uz"+i};

                quadratic_Q(i) = R(1,1);
                quadratic_Q(i+1) = R(2,2);
                quadratic_Q(i+2) = R(3,3);
            end

            if (phase == 1 || phase == 2 || phase == 4)
                %setting up constraints for rendezvous
                % dynamics constraints = # of control inputs
                % x0 = xbar0 => 1 constraints
                % make sure all u's are less than ubar => 2*(# control inputs)
                constr_dim = n_horizon - 1 + n_horizon-1 + 1 + 2*(n_horizon-1);
                constraints = zeros(constr_dim,9*n_horizon-3);
            else
                %setting up constraints for docking
                % make sure all x's are less than vbar => 2*(# x inputs)
                % linear LOS constraints => # x inputs
                constr_dim = n_horizon-1 + n_horizon-1 + 1 + 2*(n_horizon-1) + 2*n_horizon + n_horizon;
                constraints = zeros(constr_dim, 9*n_horizon-3);
            end

            for i = 1:n_horizon-1
                %adding constraints 
                % Ax_k + Bu_k = x_k+1
                location_x = 6*(i-1)+1;
                constraints(i,location_x:location_x+5) = 1;
                location_u = 6*n_horizon + 3*i + 1;
                constraints(i,location_u:location_u+2) = 1; 

            end

            constraints(constr_dim,1) = 1;


            model.varnames = names;
            model.Q = sparse(quadratic_Q);
            model.modelsense = 'min';


        end
        function us = optimizeNonlinear()
        end
        function u = mpc_estimate()
        end
    end
end