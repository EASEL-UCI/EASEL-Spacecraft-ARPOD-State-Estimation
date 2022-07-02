classdef ChaserKF
    properties (Constant)
    end
    methods (Static)
        function [x_pred, cov_pred] = predict(A,B,x_k, u_k, cov_k, R)
            x_pred = A*x_k + B*u_k;
            cov_pred = A*cov_k*transpose(A) + R;
        end
        function [z_meas,K_gain] = convertMeas(C,Q,x_k, cov_pred)
            z_meas = C*x_k;
            K_gain = cov_pred*transpose(C)*inv(C*cov_pred*transpose(C) + Q);
        end
        function [x_next, cov_next] = correct(x_pred, cov_pred, z_meas, z, K_gain, C)
            x_next = x_pred + K_gain*(z - z_meas);
            cov_next = (eye - K_gain*C)*cov_pred;
        end
    end
end