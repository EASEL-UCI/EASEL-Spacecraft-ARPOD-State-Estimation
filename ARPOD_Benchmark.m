classdef ARPOD_Benchmark
    properties (Constant)
        t_e = 14400; % eclipse time (in seconds)
        t_f = 43200; % total mission duration (in seconds)
        rho_r = 1; % maximum distance for range measurements (1 km)
        %rho_r = 50;
        rho_d = 0.1; % docking phase initial radius (0.1 km)
        %rho_d = 25;
        m_t = 2000; % mass of target (2000 kg)
        m_c = 500; % mass of chaser (500 kg)
        mu = 398600.4; %earth's gravitational constant (in km^3/s^2)
        a = 42164; % semi-major axis of GEO (42164 km)
        %a = 1000;
        Vbar = 5 * 10.^(-5); % max closing velocity while docking (in km/s)
        theta = 60; % LOS Constraint angle (in degrees)
        c = [-1;0;0]; % LOS cone direction
        x_docked = [0;0;0;0;0;0]; % docked position in km, and km/s
        x_relocation = [0;20;0;0;0;0]; %relocation position in km, km/s
        x_partner = [0;30;0;0;0;0]; %partner position in km, km/s
        ubar = 0.1;
        % can choose to add noise separately
    end
    methods (Static)
        function inLOS = isInsideLOS(traj)
            theta1 = ARPOD_Benchmark.theta * pi / 180; %docking angle in radians
            theta2 =  theta1;
            LOS_mtx = [ sin(theta1/2), cos(theta1/2), 0; 
                    sin(theta1/2), -cos(theta1/2), 0;
                    sin(theta2/2), 0, cos(theta2/2);
                    sin(theta2/2), 0, -cos(theta2/2)];
            xyz = [traj(1);traj(2);traj(3)];
            b = LOS_mtx*xyz;
            if b(1) <= 1e-5 && b(2) <= 1e-5 && b(3) <= 1e-5 && b(4) <= 1e-5
                inLOS = 1;
            else
                inLOS = 0;
            end
        end
        function inLOS = isInsideNonlinearLOS(traj)
            theta = ARPOD_Benchmark.theta * pi / 180; %docking angle in radians
            rho_d = ARPOD_Benchmark.rho_d;
            c = rho_d * ARPOD_Benchmark.c;
            rho = [traj(1);traj(2);traj(3)];
            
            dot = (rho.' * c) / (norm(rho)*norm(c));
            if (dot(1) >= cos(theta/2))
                % within LOS
                inLOS = 1;
            else
                % not within LOS
                inLOS = 0;
            end
        end
        function phase = calculatePhase(traj, reached)
            norm = traj(1:3,:);
            norm = sqrt(sum(norm.^2));
            if (reached == 0)
                if (norm > ARPOD_Benchmark.rho_r)
                    % ARPOD phase 1: Rendezvous w/out range
                    phase = 1;
                elseif norm > ARPOD_Benchmark.rho_d
                    % ARPOD phase 2: Rendezvous with range
                    phase = 2;
                else 
                    %ARPOD phase 3: Docking
                    if (ARPOD_Benchmark.isInsideLOS(traj) == 0)
                        phase = 2;
                    else
                        phase = 3;
                    end
                end
            else
                % ARPOD phase 4: Rendezvous to new location
                phase = 4;
            end
        end
        function traj = nextStep(traj0, u, timestep, noise, options)
            if (options == 1)
                % discrete control input
                u0 = @(t) u;
                [ts, trajs] = nonlinearChaserDynamics.simulateMotion(traj0, ARPOD_Benchmark.a, u0, timestep, 0);
                traj = trajs(length(ts),:);
            elseif (options == 2)
                % discrete impulsive control input. instantaneous change in
                % velocity.
                traj0 = traj0 + [0;0;0;u];
                [ts, trajs] = nonlinearChaserDynamics.simulateMotion(traj0, ARPOD_Benchmark.a,@() [0;0;0], timestep, 0);
                traj = trajs(length(ts),:);
            elseif (options == 3)
                % continuous control input
                [ts, trajs] = nonlinearChaserDynamics.simulateMotion(traj0, ARPOD_Benchmark.a, u,timestep, 0);
                traj = trajs(length(ts),:);
            end
            traj = traj.' + noise();
        end
        function sensor = sensor(state, noise, phase)
            if (phase == 1)
                %phase 1: only using bearing measurements
                sensor = ARPOD_Sensing.measure(state);
                sensor = sensor(1:2,:);
                w = noise();
                v = w(1:2,:);
            elseif (phase == 2)
                %phase 2: bearing measurements + range measurement
                sensor = ARPOD_Sensing.measure(state);
                v = noise();
            elseif (phase == 3)
                %phase 3: same as phase 2
                sensor = ARPOD_Sensing.measure(state);
                v = noise();
            elseif (phase == 4)
                %phase 4: relative phase 2 to partner spacecraft
                r = ARPOD_Benchmark.x_partner - [state(1);state(2);state(3)]; % relative position to partner spacecraft
                sensor = ARPOD_Sensing.measure(r);
                v = noise();
            end
            sensor = sensor + v;
        end
        function jacobian = jacobianSensor(state, options, r)
            x = state(1);
            y = state(2);
            z = state(3);
            if (options == 1)
                jacobian = ARPOD_Sensing.jacobianMeasurement(x,y,z);
                jacobian = jacobian(1:2,:);
            elseif (options == 2)
                jacobian = ARPOD_Sensing.jacobianMeasurement(x,y,z);
            elseif (options == 3)
                jacobian = ARPOD_Sensing.jacobianMeasurement(x,y,z);
            elseif (options == 4)
                jacobian = ARPOD_Sensing.jacobianPartner(x,y,z,r);
            end
        end
        function [A,B] = linearDynamics(T)
            mu_GM = 398600.4; %km^2/s^2;
            R = ARPOD_Benchmark.a;

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
    end
end