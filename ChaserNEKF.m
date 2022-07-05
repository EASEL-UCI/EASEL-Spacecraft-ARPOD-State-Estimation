classdef ChaserNEKF
    %{
        This is the discrete HCW EKF
        Prediction is based on discrete matrices Ax+Bu
        Measurement is based on using jacobian with nonlinear functions.

        x_t+1 = f(x_t,u_t)
        c_t = h(x_t)
    %}
    properties (Constant)
        %
        mu_GM = 398600.4; %km^2/s^2;
    end
    methods (Static)
        function [state,cov] = prediction(state0, cov0, u0, T, R, systemCov)
            n = sqrt(ChaserEKF.mu_GM / (R.^3) );
            
        end
        function [state,cov] = estimate(state_t,cov_t,u_t,T,R,z_t,systemCov,measCov)
            [state_pred, cov_pred] = ChaserEKF.prediction(state_t,cov_t,u_t,T,R,systemCov);
            H = ARPOD_Sensing.jacobianMeasurement(state_pred(1),state_pred(2),state_pred(3)); %jacobian matrix
            K_gain = cov_pred*transpose(H)*inv(H*cov_pred*transpose(H) + measCov);

            state = state_pred + K_gain*(z_t-ARPOD_Sensing.measure(state_pred));
            cov = (eye(6) - K_gain*H)*cov_pred;
        end
    end
end