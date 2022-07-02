
%3DOF ARPOD sensing model
%assumes the trajectories are size 6 with position and their derivatives.
classdef ARPOD_Sensing
    properties (Constant)

    end
    methods (Static)
        function sense_data = convertTrajs(trajs)
            [n_traj, dim_traj] = size(trajs);
            e = zeros(3,n_traj);
            for i = 1:n_traj
                traj = trajs(i,:); 
                x = traj(1);
                y = traj(2);
                z = traj(3);
                norm = sqrt(x*x+y*y+z*z);
                e1 = atan(y/x);
                e2 = asin(z/norm);
                e3 = norm;
                e(:,i) = [e1;e2;e3];
            end
            sense_data = e;
        end
        function noisy_data = noisifyData(sense_data, noise_model)
            [dim_traj, n_traj] = size(sense_data);
            noisy_data = sense_data;
            for i = 1:n_traj
                noisy_data(:,i) = sense_data(:,i) + noise_model();
            end
            %noisy_trajs is returned
        end
    end
end