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
            names = cell(1,n_horizon + (n_horizon-1));
            dim = n_horizon + n_horizon - 1;
            for i = 1:dim
                
            end
            if (phase == 3)

            else

            end
        end
        function us = optimizeNonlinear()
        end
        function us = optimizeConvex()
        end
        function u = mpc_estimate()
        end
    end
end