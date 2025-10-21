clc
clear all
close all

v_max = 25;
a_max = 100;

% P1 = [0;0;356.521686];
% P2 = [-25;-75;400];
% P3 = [30; 40;400];
% P4 = [-10; 0;420];
% P5 = [-30; 40;380];

P1 = [0;0;356.521686];
P2 = [10;30;430];
P3=  [75;-50;400];

P15 = (P1+P2)/2;
P25 = (P2+P3)/2;
P35 = (P1+P3)/2;

P = [P1 P2 P3 P1]';
Ps = size(P);

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
q_all1 = [];
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

    q_all1 = [q_all1; q_seg];
    q_vel_all = [q_vel_all; v_seg];
    q_acc_all = [q_acc_all; a_seg];
end

% Append final point
time1 = [time; t3(end)];
q_all1 = [q_all1; Ps(end,:)];
q_vel_all1 = [q_vel_all; zeros(1, num_joints)];
q_acc_all1 = [q_acc_all; zeros(1, num_joints)];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
P = [P1 P15 P2 P25 P3 P35 P1]';
Ps = size(P);

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
time = [time; t3(end)];
q_all2 = [q_all; Ps(end,:)];
q_vel_all = [q_vel_all; zeros(1, num_joints)];
q_acc_all = [q_acc_all; zeros(1, num_joints)];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% === Final Plot: One figure per joint with 3 subplots ===
figure;
for j = 1:num_joints
    subplot(num_joints, 3, 3*(j-1)+1);
    plot(time1, q_all1(:,j), 'b', 'LineWidth', 1.5);
    ylabel(sprintf('q_%d [unit]', j)); grid on;
    if j == num_joints
        xlabel('Time [s]');
    end
    title(sprintf('Joint %d Position', j));

    subplot(num_joints, 3, 3*(j-1)+2);
    plot(time1, q_vel_all1(:,j), 'r', 'LineWidth', 1.5);
    ylabel(sprintf('dq_%d [unit/s]', j)); grid on;
    if j == num_joints
        xlabel('Time [s]');
    end
    title(sprintf('Joint %d Velocity', j));

    subplot(num_joints, 3, 3*(j-1)+3);
    plot(time1, q_acc_all1(:,j), 'g', 'LineWidth', 1.5);
    ylabel(sprintf('ddq_%d [unit/s^2]', j)); grid on;
    if j == num_joints
        xlabel('Time [s]');
    end
    title(sprintf('Joint %d Acceleration', j));
end

% === Simulink Export ===
simin.time = time;
simin.signals.values = q_all1;

%plotting
% Number of time steps
N = size(q_all1, 1);
q_all1 = q_all1';
for i = 1:N
    s1 = q_all1(1, i);
    s2 = q_all1(2, i);
    s3 = q_all1(3, i);
    [x, y, z] = Delta_forward_kinematics(s1, s2, s3);
    pos_cartesian2(:, i) = [x, y, z];
end

% Number of time steps
N = size(q_all2, 1);
q_all2 = q_all2';
for i = 1:N
    s1 = q_all2(1, i);
    s2 = q_all2(2, i);
    s3 = q_all2(3, i);
    [x, y, z] = Delta_forward_kinematics(s1, s2, s3);
    pos_cartesian3(:, i) = [x, y, z];
end

figure 

plot3(pos_cartesian2(1,:),pos_cartesian2(2,:),pos_cartesian2(3,:),Linestyle=":",LineWidth=2,Color=[0 0 0])
hold on
plot3(pos_cartesian3(1,:),pos_cartesian3(2,:),pos_cartesian3(3,:),Linestyle="-",LineWidth=2,Color=[0 0 0])

plot3(P1(1), P1(2), P1(3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'magenta'); % Start point
plot3(P2(1), P2(2), P2(3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'cyan'); % End point
plot3(P3(1), P3(2), P3(3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'yellow'); % End point
plot3(P15(1), P15(2), P15(3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'blue'); % End point
plot3(P25(1), P25(2), P25(3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'red'); % End point
plot3(P35(1), P35(2), P35(3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'green'); % End point
grid on

% Axes labels with units
xlabel('X in mm');
ylabel('Y in mm');
zlabel('Z in mm');
legend('Trajectory1','Trajectory2','P1','P2','P3','P15','P25','P35');

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

