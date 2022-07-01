classdef ChaserEKF
    properties (Constant)
    end
    methods (Static)
        function [x_pred, cov_pred] = predict(pred_f,gpred_f, B, x_k, u_k, cov_k, R)
            x_pred = pred_f(x_k) + B*u_k;
            G = gpred_f(x_k,u_k);
            cov_pred = G*cov_k*transpose(G) + R;
        end
        function [z_meas,K_gain] = convertMeas(fMeas, gfMeas, c, cov_pred)
            z_meas = fMeas(c);
            H = gfMeas(c);
            K_gain = cov_pred*tranpose(H)*inv(H*cov_pred*transpose(H)+Q);
        end
        function [x_next, cov_next] = correct(x_pred, cov_pred, c,z_meas, K_gain)
            x_next = x_pred + K_gain*(c - z_meas);
            cov_next = (eye - K_gain*H)*cov_pred;
        end
    end
end