classdef ChaserPF
    properties
        particles
        weights
        n_particles
        ESS_Threshold
        state
    end
    methods (Static)
        function rand = sample_gaussian(mu,cov)
            rand = mvnrnd(mu,cov,1);
        end
        function indexes = sample_particles(weights)
            disp(weights);
            [N,dim] = size(weights);
            indexes = randsample(N,N,true,weights.');
        end
    end
    methods 
        function obj = initPF(obj, mean,cov, n, ESS_Threshold)
            %{
                Initialize particle filter:
                    acts as a constructor
                Adds in number of particles
                particles themselves
                and the weights.
            %}
            obj.n_particles = n;
            obj.particles = mvnrnd(mean, cov, n).';
            obj.weights = ones(n,1)/n;
            obj.ESS_Threshold = ESS_Threshold;
            obj.state = mean.';
        end
        function obj = resampling(obj)
            %{
            %}
            ESS = 1/sum(obj.weights.^2);
            if (ESS < obj.ESS_Threshold)
                %resample
                old_particles = obj.particles;
                idx_particles = ChaserPF.sample_particles(obj.particles);
                for i =1:obj.n_particles
                    j = idx_particles(i);
                    obj.particles(:,i) = old_particles(:,j);
                end
                obj.weights = ones(N,1)/N;
            end
        end        
        function state = estimateState(obj)
            state = zeros(6,1);
            for i = 1:obj.n_particles
                state = state + obj.weights(i) * obj.particles(:,i);
            end
        end

        function obj = particlesUpdate(obj, measurement, Q_cov, R_cov, u, tstep, phase)
            [dim,N] = size(obj.particles);

            idx_particles = ChaserPF.sample_particles(obj.weights);
            new_particles = zeros(dim,N);
            new_weights = zeros(N,1);
            sum_weights = 0;

            if phase == 1
                R_cov = R_cov(1:2,1:2);
            end

            for i = 1:N
                j = idx_particles(i);
                prop = ARPOD_Benchmark.nextStep(obj.particles(:,j), u,tstep,@() [0;0;0;0;0;0], 1);
                new_particles(:,i) = ChaserPF.sample_gaussian(prop,Q_cov);

                estimated_measurement = ARPOD_Benchmark.sensor(prop, @() [0;0;0], phase);
                new_weights(i) = abs(mvnpdf(measurement, estimated_measurement, R_cov));
                sum_weights = sum_weights + new_weights(i);
            end
            new_weights = new_weights / sum_weights;

            obj.particles = new_particles;
            obj.weights = new_weights;
        end
        function obj = estimate(obj, u, measurement, Q_cov, R_cov, tstep, phase)
            obj = obj.resampling();
            obj = obj.particlesUpdate(measurement, Q_cov, R_cov, u ,tstep, phase);
            obj.state = obj.estimateState();
        end
    end
end