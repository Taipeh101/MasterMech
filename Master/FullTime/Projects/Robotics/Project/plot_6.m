clear all
clc
close all

P1 = [0;0;356.521686];
P2 = [10;30;430];
P3=  [75;-50;400];

% P0=[0;0;356.521686];
% P1=[-25;-75;400];
% P2=[75;-50;400];
% P3=[25;75;400];

P15 = [(P1(1)+P2(1))/2;(P1(2)+P2(2))/2;(P1(3)+P2(3))/2];
P25 = [(P3(1)+P2(1))/2;(P3(2)+P2(2))/2;(P3(3)+P2(3))/2];
P35 = [(P1(1)+P3(1))/2;(P1(2)+P3(2))/2;(P1(3)+P3(3))/2];

v_max = 50;
a_max = 100;


[pos_jointspace1, time1,v,a] = Delta_MoveJ(P1, P2, v_max, a_max);
[pos_jointspace2, time2,v,a] = Delta_MoveJ(P2, P3, v_max, a_max);
[pos_jointspace3, time3,v,a] = Delta_MoveJ(P3, P1, v_max, a_max);

pos_jointspace = [pos_jointspace1 pos_jointspace2 pos_jointspace3];
t_jointspace = [time1 time2+time1(end)  time3+time2(end)+time1(end)];

% Number of time steps
N = size(pos_jointspace, 2);

for i = 1:N
    s1 = pos_jointspace(1, i);
    s2 = pos_jointspace(2, i);
    s3 = pos_jointspace(3, i);
    [x, y, z] = Delta_forward_kinematics(s1, s2, s3);
    pos_cartesian(:, i) = [x, y, z];
end

[pos_taskspace1,v1,a1, time1] = Delta_MoveL(P1, P2,v_max,a_max);
[pos_taskspace2,v2,a2, time2] = Delta_MoveL(P2, P3,v_max,a_max);
[pos_taskspace3,v3,a3, time3] = Delta_MoveL(P3, P1,v_max,a_max);

pos_taskspace = [pos_taskspace1 pos_taskspace2 pos_taskspace3];
t_taskspace = [time1 time2+time1(end)  time3+time2(end)+time1(end)];

%==========================================================================
P = [P1 P15 P2 P25 P3 P35 P1]';
% P = [P1 P2 P3 P1]';
Ps = size(P);

v_max = 50/2;
a_max = 100;

for i = 1:length(P)
    [Ps(i,1),Ps(i,2),Ps(i,3)] = Delta_inverse_kinematics(P(i,1),P(i,2),P(i,3));
end


Diff_max = zeros(length(Ps)-1,1);

for i = 2:length(Ps)
    Diff(i-1,:) = (Ps(i,:)-Ps(i-1,:));
    Diff_max(i-1) = max(abs(Diff(i-1,:)));
end
sign_array = sign(Diff);

t1 = v_max/a_max;
t2 = Diff_max/v_max;
t3 = t1 + t2;

for i = 2:length(t3)
    t3(i) = t3(i)+t3(i-1);
end
t3 = [0; t3];
velocity_vector = zeros(size(Ps));

for i = 2:length(velocity_vector)-1
    velocity_vector(i,:) = (Ps(i+1,:)-Ps(i-1,:))./(t3(i+1)-t3(i-1));
end

% === Generate cubic trajectories ===
num_joints = size(Ps,2);
dt = 0.01;
coeffs = cell(size(Ps,1)-1, num_joints);

time = [];
q_all = [];
q_vel_all = [];
q_acc_all = [];

for i = 1:size(Ps,1)-1
    t_i = t3(i);
    t_ip1 = t3(i+1);
    t_segment = t_i:dt:t_ip1;
    t_vals = t_segment(1:end-1)';  % column vector
    time = [time; t_vals];

    q_seg = zeros(length(t_vals), num_joints);
    v_seg = zeros(length(t_vals), num_joints);
    a_seg = zeros(length(t_vals), num_joints);

    for j = 1:num_joints
        q_i = Ps(i,j);
        q_ip1 = Ps(i+1,j);
        dq_i = velocity_vector(i,j);
        dq_ip1 = velocity_vector(i+1,j);

        % Solve cubic polynomial
        T = [1, t_i,    t_i^2,    t_i^3;
             0, 1,      2*t_i,    3*t_i^2;
             1, t_ip1,  t_ip1^2,  t_ip1^3;
             0, 1,      2*t_ip1, 3*t_ip1^2];
        Y = [q_i; dq_i; q_ip1; dq_ip1];
        a = T \ Y;
        coeffs{i,j} = a;

        % Evaluate position, velocity, acceleration
        q_seg(:,j) = a(1) + a(2)*t_vals + a(3)*t_vals.^2 + a(4)*t_vals.^3;
        v_seg(:,j) = a(2) + 2*a(3)*t_vals + 3*a(4)*t_vals.^2;
        a_seg(:,j) = 2*a(3) + 6*a(4)*t_vals;
    end

    q_all = [q_all; q_seg];
    q_vel_all = [q_vel_all; v_seg];
    q_acc_all = [q_acc_all; a_seg];
end

% Append final point
t_continuous = [time; t3(end)];
q_all = [q_all; Ps(end,:)]';
q_vel_all = [q_vel_all; zeros(1, num_joints)];
q_acc_all = [q_acc_all; zeros(1, num_joints)];

% Number of time steps
N = size(q_all, 2);

for i = 1:N
    s1 = q_all(1, i);
    s2 = q_all(2, i);
    s3 = q_all(3, i);
    [x, y, z] = Delta_forward_kinematics(s1, s2, s3);
    pos_cartesian2(:, i) = [x, y, z];
end

%==========================================================================

figure
plot3(pos_cartesian(1,:),pos_cartesian(2,:),pos_cartesian(3,:),Linestyle="-",LineWidth=2,Color=[0 0 0])
hold on
plot3(pos_taskspace(1,:),pos_taskspace(2,:),pos_taskspace(3,:),Linestyle="--",LineWidth=2,Color=[0 0 0])
plot3(pos_cartesian2(1,:),pos_cartesian2(2,:),pos_cartesian2(3,:),Linestyle=":",LineWidth=2,Color=[0 0 0])

% Plot points P1, P2, P3 with different markers
plot3(P1(1), P1(2), P1(3), 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName', 'P1');
plot3(P15(1), P15(2), P15(3), 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'magenta', 'DisplayName', 'P15');
plot3(P2(1), P2(2), P2(3), 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', 'P2');
plot3(P25(1), P25(2), P25(3), 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'yellow', 'DisplayName', 'P25');
plot3(P3(1), P3(2), P3(3), 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'b', 'DisplayName', 'P3');
plot3(P35(1), P35(2), P35(3), 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'cyan', 'DisplayName', 'P35');

grid on
% Axes labels with units
xlabel('X in mm');
ylabel('Y in mm');
zlabel('Z in mm');
legend('JointSpace','TaskSpace','Continuous','P1','P15','P2','P25','P3','P35');

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