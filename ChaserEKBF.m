classdef ChaserK
    %This is Continuous Extended Kalman Bucy Filter (continuous version of
    %kalman filter

    % xdot = f(xt,ut)
    % zt = h(xt)
    properties (Constant)

    end
    methods (Static)
        function [stateDot, covDot] = dynamicsEstimation(state_t, cov_t, u_t, tstart,tfinal)
            % Do this later...
            % Basically you have to implement time-varied dynamics f, its
            % jacobian
            % also implement time-varied measurements
            % and their jacobian
            % it's a whole different ballgame
        end
    end
end