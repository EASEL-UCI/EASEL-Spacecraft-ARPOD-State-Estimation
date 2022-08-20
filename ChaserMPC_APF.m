classdef ChaserMPC_APF
    methods (Static)
        function cost = setupOriginalAPF(traj, obstacle, kO, KO)
            xyz = [traj(1); traj(2); traj(3)];
            xyzO = obstacle(1:3,:);
            dO = obstacle(4,:);
            dist = norm(xyz-xyzO);
            if dist <= dO
                cost = 0.5*KO*(norm(xyz-xyzO) - kO * dO).^2;
            else
                cost = 0;
            end
        end
        function costF = costAPF(Q,R, n_horizon, obstacles, KO, kO)
            %{
                Obstacle: (x,y,z, distance)
            %}
            [H,f] = ChaserMPC.setupQuadraticCost(Q,R,n_horizon);
            %APF
            [dim,n_obstacles] = size(obstacles);
            function cost = costfun(traj)
                cost = 0;
                for i = 1:n_obstacles
                    cost = cost + ChaserMPC_APF.setupOriginalAPF(traj, obstacles(:,i), kO, KO);
                end
            end
            costF = @costfun;
        end
    end
end