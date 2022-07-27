classdef ChaserEKF
    %{
        This is the discrete HCW EKF
        Prediction is based on discrete matrices Ax+Bu
        Measurement is based on using jacobian with nonlinear functions.

        x_t+1 = Ax_t + Bu_t
        c_t = h(x_t)
    %}
    properties (Constant)
        %
        mu_GM = 398600.4; %km^2/s^2;
    end
    methods (Static)
        function [state,cov] = prediction(state0, cov0, u0, T, R, systemCov)
            n = sqrt(ChaserEKF.mu_GM / (R.^3) );
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

            state = A*state0 + B*u0;
            cov = A*cov0*transpose(A) + systemCov;
        end
        function [state,cov] = estimate(state_t,cov_t,u_t,T,R,z_t,systemCov,measCov, phase)
            [state_pred, cov_pred] = ChaserEKF.prediction(state_t,cov_t,u_t,T,R,systemCov);
            H = ARPOD_Benchmark.jacobianSensor(state_pred,phase, ARPOD_Benchmark.x_partner);
            K_gain = cov_pred*transpose(H)*inv(H*cov_pred*transpose(H) + measCov);

            measure = ARPOD_Benchmark.sensor(state_pred, @() [0;0;0], phase);
            state = state_pred + K_gain*(z_t-measure);
            cov = (eye(6) - K_gain*H)*cov_pred;
        end
    end
end