theta1 = 60 * pi /180;
theta2 = theta1;

set(groot,'defaultAxesTickLabelInterpreter','latex');  
set(groot,'defaulttextinterpreter','latex');
set(groot,'defaultLegendInterpreter','latex');

fsize = 20; % figure fontsize
lw = 2; % linewidth

figure(1)
%draw phase 2 sphere boundary
[x,y,z] = sphere(10);
r = ARPOD_Benchmark.rho_r;
scatter3(r*x(:),r*y(:),r*z(:), 5,'o', 'filled', 'MarkerEdgeColor', 'c');
axis equal
hold on

%draw phase 3 sphere boundary
[x,y,z] = sphere(5);
r = ARPOD_Benchmark.rho_d;
scatter3(r*x(:),r*y(:),r*z(:), 5, 'o', 'filled', 'MarkerEdgeColor', 'm');

c = ARPOD_Benchmark.rho_d;
%drawing pillars of pyramid
%plot3([0,-sin(theta2/2)*c], [0,-sin(theta1/2)*c], [0,-cos(theta1/2)*c], 'g');
plot3([0,-cos(theta1/2)*c], [0,-sin(theta2/2)*c], [0,-sin(theta1/2)*c], 'g');
%plot3([0,-sin(theta2/2)*c], [0,sin(theta1/2)*c], [0,-cos(theta1/2)*c], 'g');
plot3([0,-cos(theta1/2)*c], [0,sin(theta1/2)*c], [0,-sin(theta2/2)*c], 'g');

%plot3([0,sin(theta2/2)*c], [0,-sin(theta1/2)*c], [0,-cos(theta1/2)*c], 'g');
plot3([0,-cos(theta1/2)*c], [0,-sin(theta1/2)*c],[0,sin(theta2/2)*c],  'g');

%plot3([0,sin(theta2/2)*c], [0,sin(theta1/2)*c], [0,-cos(theta1/2)*c], 'g');
plot3([0,-cos(theta1/2)*c], [0,sin(theta1/2)*c], [0,sin(theta2/2)*c],  'g');

%drawing base of pyramid
%plot3([-sin(theta2/2)*c, sin(theta2/2)*c], [-sin(theta1/2)*c,-sin(theta1/2)*c], [-cos(theta1/2)*c,-cos(theta1/2)*c], 'g');
plot3([-cos(theta1/2)*c,-cos(theta1/2)*c], [-sin(theta1/2)*c,-sin(theta1/2)*c],[-sin(theta2/2)*c, sin(theta2/2)*c], 'g');

%plot3([-sin(theta2/2)*c, sin(theta2/2)*c], [sin(theta1/2)*c, sin(theta1/2)*c], [-cos(theta1/2)*c,-cos(theta1/2)*c], 'g');
plot3([-cos(theta1/2)*c,-cos(theta1/2)*c],[sin(theta1/2)*c, sin(theta1/2)*c], [-sin(theta2/2)*c, sin(theta2/2)*c],'g');

%plot3([-sin(theta2/2)*c, -sin(theta2/2)*c], [-sin(theta1/2)*c,sin(theta1/2)*c], [-cos(theta1/2)*c,-cos(theta1/2)*c], 'g');
plot3([-cos(theta1/2)*c,-cos(theta1/2)*c], [-sin(theta1/2)*c,sin(theta1/2)*c], [-sin(theta2/2)*c, -sin(theta2/2)*c], 'g');

%plot3([sin(theta2/2)*c, sin(theta2/2)*c], [-sin(theta1/2)*c, sin(theta1/2)*c], [-cos(theta1/2)*c,-cos(theta1/2)*c], 'g');
plot3([-cos(theta1/2)*c,-cos(theta1/2)*c],[-sin(theta1/2)*c, sin(theta1/2)*c], [sin(theta2/2)*c, sin(theta2/2)*c], 'g');

%draw truechaserTraj
plot3(trackEKFTrueTraj(1,:), trackEKFTrueTraj(2,:), trackEKFTrueTraj(3,:), 'r');
%draw estchaserTraj
plot3(trackEKFTraj(1,:), trackEKFTraj(2,:), trackEKFTraj(3,:), 'b');
hold off

%draw target position
%draw chaserStart
%draw chaserEnd
%draw 
title("Chaser Trajectory")
xlabel("x [km]")
ylabel("y [km]")
zlabel("z [km]")
set(gca,'fontsize',fsize)
set(gcf,'Position', [10 10 500 500])
set(gca, 'TickLabelInterpreter', 'latex')
