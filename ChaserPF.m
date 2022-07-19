classdef ChaserPF
    properties (Constant)
    end
    methods (Static)
        function [weights, particles] = initialize_particles(mean, cov, n)
            particles = mvnrnd(mean,cov,n);
            weights = ones(n,1)/n;
        end
        function rand = sample_gaussian(mu,cov)
            rand = mvrnd(mu,cov,1);
        end
        function indexes = sample_particles(weights)
            N = size(weights);
            indexes = randsample(N,N,true,weights);
        end
        function [weights, particles] = nonlinearPF(weights0, particles0, measurement, ESS_threshold, Q_cov, R_cov, u, R, T)
            [dim, N] = size(particles0);
            %calculate ESS
            ESS = 0;
            for i=1:N
                ESS = ESS + weights0(i).^2;
            end
            ESS = 1/ESS;
            if (ESS < ESS_threshold)
                % if effective sample size is below desired threshold,
                % resample particles.
                old_particles = particles0;
                idx_particles = ChaserPF.sample_pareticles(weights0);
                for i = 1:N
                    j = idx_particles(i);
                    particles0(:,i) = old_particles(:,j);
                end
                weights0 = zeros(N) + 1/N;
            end


            %initialize particle and weights;
            %discretely sample particle from discrete weight distribution
            idx_particles = ChaserPF.sample_particles(weights0);
            particles = zeros(dim,N);
            weights = zeros(N);
            sum_weights = 0;
            for i = 1:N
                j = idx_particles(i);

                %propagate individual particle
                [ts, traj] = nonlinearChaserDynamics( particles0(:,j), R, u, T, T/2);
                prop = transpose(traj(3,:));

                %sampling particle from transition distribution
                particles(:,i) = ChaserPF.sample_gaussian(prop,Q_cov);

                %create weight using measurement likelihood distribution\
                expected_measurement = ARPOD_Sensing.measure(prop);
                weights(i) = ChaserPF.sample_gaussian(measurement - expected_measurement, R_cov);
                sum_weights = sum_weights + weights(i);
            end
            weights = weights / sum_weights; %normalization of weights
        end
        function state = estimateState(weights, particles)
            N = size(weights);
            state = zeros(6,1);
            for i = 1:N
                state = state + weights(i) * particles(:,i);
            end
        end
    end
end