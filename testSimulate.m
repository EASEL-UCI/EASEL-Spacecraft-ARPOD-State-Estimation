traj0 = [-100;-100;-100;0;0;0]; %prev_traj
R = 10000;
u0 = [0;0;0]; % u_prev
T = 1;
[ts,trajs] = nonlinearChaserDynamics.simulateMotion(traj0,R,u0,T);
%traj(:,n)
%traj_list
% compare EKF estimation list with traj_list


figure()
plot3(trajs(:,1), trajs(:,2), trajs(:,3));
title('Chaser Trajectory')
xlabel('x')
ylabel('y')
zlabel('z')
grid on

figure()
plot(ts, trajs(:,3), '-b')
title('chaser xyz')

hold on
plot(ts, trajs(:,1), '-r')

plot(ts, trajs(:,2), '-g')
hold off

