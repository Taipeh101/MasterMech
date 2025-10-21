clc
close all
clear all

v_max = 50;
a_max = 100;

P2 = [10;30;400];
P1 = [0;0;356.521686];
P3=  [75;-50;400];

[pos_jointspace1, time1,v,a] = Delta_MoveJ(P1, P2, v_max, a_max);
[pos_jointspace2, time2,v,a] = Delta_MoveJ(P2, P3, v_max, a_max);

pos_jointspace = [pos_jointspace1 pos_jointspace2];
time = [time1 time2+time1(end)];


figure
plot(time, pos_jointspace)

simin.time = time';
simin.signals.values = pos_jointspace';


%%

% Number of time steps
N = size(pos_jointspace, 2);

% Preallocate Cartesian position array
pos_cartesian = zeros(3, N);

% Convert joint space to Cartesian positions using forward kinematics
for i = 1:N
    s1 = pos_jointspace(1, i);
    s2 = pos_jointspace(2, i);
    s3 = pos_jointspace(3, i);
    [x, y, z] = Delta_forward_kinematics(s1, s2, s3);
    pos_cartesian(:, i) = [x, y, z];
end

% Compute unit direction vector of the desired straight line
line_vec = P2 - P1;
line_unit = line_vec / norm(line_vec);

% Compute deviation from the straight line at each timestep
deviation = zeros(1, N);
for i = 1:N
    p = pos_cartesian(:, i);
    vec_to_point = p - P1;
    proj_length = dot(vec_to_point, line_unit);
    proj_point = P1 + proj_length * line_unit;
    deviation(i) = norm(p - proj_point);
end

% Plot the deviation
figure;
plot(time, deviation,Linestyle="-",LineWidth=1.3,Color=[0 0 0]);
xlabel('Time [s]');
ylabel('Deviation from straight line [mm]');
grid on;
plot_properties([0 0.5 2.5],[0 1 3]);

figure
plot3(pos_cartesian(1,:),pos_cartesian(2,:),pos_cartesian(3,:),Linestyle="-",LineWidth=1.3,Color=[0 0 0]);
hold on
plot3([P1(1), P2(1)], [P1(2), P2(2)], [P1(3), P2(3)],Linestyle="--",LineWidth=1.3,Color=[0 0 0]);
plot3([P2(1), P3(1)], [P2(2), P3(2)], [P2(3), P3(3)],Linestyle=":",LineWidth=1.3,Color=[0 0 0]);
grid on
% Plot start and end points
plot3(P1(1), P1(2), P1(3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'black'); % Start point
plot3(P2(1), P2(2), P2(3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'blue'); % End point
plot3(P3(1), P3(2), P3(3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'green'); % End point
% Axes labels with units
xlabel('X in mm');
ylabel('Y in mm');
zlabel('Z in mm');
legend('Trajecotry', 'Ideal Straight Line P1-P2','Ideal Straight Line P2-P3', 'P1', 'P2', 'P3');

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