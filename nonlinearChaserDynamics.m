% gravitational constant in km^2/s^2 from chance-constr MPC

classdef nonlinearChaserDynamics
    properties (Constant)
        mu_GM = 398600.4;
    end
    methods (Static)
        function traj = ChaserMotion(t,traj0,R,u)
            %{
                Constants:
                -----------
                    mu_GM: gravitational constant in km^2/s^2
                Paramters:
                ----------
                    traj0 = initial trajectory
                    R = orbital radius of the chaser spacecraft
                    u = external thrusters [ux,uy,uz]. modeled as a
                    function
                Returns:
                --------
                    [xdot,ydot,zdot,xdotdot,ydotdot,zdotdot] = HUGE matrix of 6 functions.
                    xdot,ydot,zdot = differential equation moved to be in terms of
                    first derivatives
        
                    xdotdot, ydotdot, zdotdot = differential equations moved to be
                    in terms of second derivatives
        
                    u know if i had half a mind, i'd go for zot, zotzot, and
                    zotzotzot hehe.
            %}
            %traj is [x,y,z,xdot,ydot,zdot]
            mu_GM = nonlinearChaserDynamics.mu_GM;
            x = traj0(1);
            y = traj0(2);
            z = traj0(3);
            
            ut = u(t); %function
            ux = ut(1); %indexing thrusters
            uy = ut(2);
            uz = ut(3);

            n = sqrt(mu_GM / (R.^3)); %orbital velocity
        
            %distance formula on chaser orbital radius ^3
            %resembles gravitational formula but generalized for 3d.
            const = ((R+x).^2 + y.^2 + z.^2).^(3/2); 
            
            xdot = traj0(4);
            ydot = traj0(5);
            zdot = traj0(6);
            xdotdot = 2*n*ydot + n*n*(R+x) - mu_GM*(R+x)/const + ux;
            ydotdot = 2*n*xdot + n*n*y - mu_GM*y/const + uy;
            zdotdot = -mu_GM*z/const + uz;
        
            %return
            traj = [xdot;ydot;zdot;xdotdot;ydotdot;zdotdot];
        end
        function [ts,trajs] = simulateMotion(traj0,R,u,T)
            f = @(t,traj) nonlinearChaserDynamics.ChaserMotion(t,traj,R,u);
            t0 = 0;
            [ts,trajs] = ode45(f,[t0,T], traj0);

        end
        function noisy_trajs = noisifyMotion(trajs, noise_model)
            [n_traj, dim_traj] = size(trajs);
            noisy_trajs = trajs;
            acc_noise = zeros([1,dim_traj]); %accumulated noise
            for i = 1:n_traj
                acc_noise = acc_noise + noise_model(); %make sure that system noise is consistent.
                noisy_trajs(i,:) = trajs(i,:) + acc_noise; 
            end
            %noisy_trajs is returned
        end
    end
end


