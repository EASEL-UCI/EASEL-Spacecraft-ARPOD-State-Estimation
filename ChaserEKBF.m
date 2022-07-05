classdef ChaserEKBF
    %This is Continuous Extended Kalman Bucy Filter (continuous version of
    %kalman filter

    % xdot = f(xt,ut)
    % z_k = h(x_k) ====> we cannot take continuous time measurements (in
    % real life it would be discrete by nature).
    properties (Constant)

    end
    methods (Static)
        function covDot = covDynamics(t,state_t, cov_t, u_t, R, systemCov)
            %u_t is a function
            cov = reshape(cov_t,6,6).';
            Ft = nonlinearChaserDynamics.ChaserJacobian(t,state_t, R, u_t);
            covDot = Ft*cov + cov*transpose(Ft) + systemCov;
        end
        function [state, cov] = estimate(state_t, cov_t, u_t, T, R, z_t, systemCov, measCov)
            stateDot = @(t,traj) nonlinearChaserDynamics.ChaserMotion(t,traj, R, u_t);
            covDot = @(t,traj) reshape(ChaserEKBF.covDynamics(t,state_t,traj,u_t, R, systemCov).',[],1);
            
            %prediction stage
            t0 = 0;
            [ts,trajs] = ode45(stateDot,[t0,T],state_t);
            [n_traj, dim_traj] = size( trajs );
            pred_state = trajs(n_traj,:).';

            [ts,trajs] = ode45(covDot,[t0,T], cov_t);
            [n_traj, dim_traj] = size( trajs );
            pred_cov = reshape(trajs(n_traj,:),6,6).';

            %u_t is function
            H = ARPOD_Sensing.jacobianMeasurement(pred_state(1), pred_state(2), pred_state(3));
            K_gain = pred_cov*transpose(H)*inv(H*pred_cov*transpose(H) + measCov);

            state = pred_state + K_gain*(z_t - ARPOD_Sensing.measure(pred_state));
            cov = (eye(6) - K_gain*H)*pred_cov;
        end
    end
end