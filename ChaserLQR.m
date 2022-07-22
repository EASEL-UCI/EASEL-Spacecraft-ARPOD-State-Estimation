classdef ChaserLQR
    properties (Constant)
    end
    methods (Static)
        function [A,B] = linearDynamics(R, T)
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
        end
        function u = optimal_control(state, Q, R, R_orbit, timestep)
            % LQR does not care about any controller constraints imposed by
            % ARPOD problem. LQR should be used to help test the state
            % estimators.

            % cannot solve HJB formualtion since a matrix required needs to
            % solve Riccati Algebraic Equation

            [A,B] = ChaserLQR.linearDynamics(R_orbit, timestep);
            [K,S,e] = LQR(A,B,Q,R,zeros(3,3));
            u = -K*state;
        end
    end
end