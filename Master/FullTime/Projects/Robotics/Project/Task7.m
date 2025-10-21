clear all
clc
close all

positions_array = ['A3','J2','C3','D1','D2'];

[time, pos_jointspace] = pick_and_place_motion(positions_array,50,100,40,70,20,50);

figure
plot(time, pos_jointspace)

% Number of time steps
N = size(pos_jointspace, 2);

for i = 1:N
    s1 = pos_jointspace(1, i);
    s2 = pos_jointspace(2, i);
    s3 = pos_jointspace(3, i);
    [x, y, z] = Delta_forward_kinematics(s1, s2, s3);
    pos_cartesian(:, i) = [x, y, z];
end

simin.time = time';
simin.signals.values = pos_jointspace';

P1 = [0;0;356.521686];

figure
plot3(pos_cartesian(1,:),pos_cartesian(2,:),pos_cartesian(3,:),Linestyle="-",LineWidth=2,Color=[0 0 0])
hold on
plot3(P1(1), P1(2), P1(3), 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName', 'home');

grid on
% Axes labels with units
xlabel('X in mm');
ylabel('Y in mm');
zlabel('Z in mm');
legend('trajectory','home');

set(gca, ... 
            'XGrid', 'on', ...
            'YGrid', 'on', ...
            'GridLineStyle', '--', ...
            'LineWidth', 0.8, ...
            'GridAlpha', 0.5, ...
            'XMinorGrid', 'off' , ...
            'YMinorGrid', 'off', ...
            'MinorGridLineStyle', ':', ...
            'FontName', 'Times New Roman', ...
            'FontSize', 25, ...
            'FontAngle', 'italic')
set(gcf, 'color','w')