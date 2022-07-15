classdef ChaserPF
    properties (Constant)
    end
    methods (Static)
        function [weights, particles] = initialize_particles(mean, cov,n)
            particles = mvnrnd(mean,cov,n);
            weights = ones(n,1)/n;
        end
        function 
    end
end