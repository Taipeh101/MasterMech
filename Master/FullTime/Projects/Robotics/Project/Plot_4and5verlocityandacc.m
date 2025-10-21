clear all
clc
close all


P1 = [0;0;356.521686];
P2 = [10;30;430];
P3=  [75;-50;400];

v_max = 50;
a_max = 100;

[pos_taskspace1,v,a, t1] = Delta_MoveL(P1, P2,v_max,a_max);
[pos_taskspace2,v,a, t2] = Delta_MoveL(P2, P3,v_max,a_max);

pos_taskspace = [pos_taskspace1 pos_taskspace2];
t = [t1 t2+t1(end)];

% Transpose task space position to get N×3 format
pos_taskspace = pos_taskspace';  % Now N×3

% Time step (assumed constant)
dt = t(2) - t(1);
N = size(pos_taskspace, 1);

% Preallocate joint space arrays
pos_jointspace = zeros(N, 3);
vel_jointspace = zeros(N, 3);
acc_jointspace = zeros(N, 3);

% --- Inverse kinematics: compute joint positions
for i = 1:N
    x = pos_taskspace(i, 1);
    y = pos_taskspace(i, 2);
    z = pos_taskspace(i, 3);
    [s1, s2, s3] = Delta_inverse_kinematics(x, y, z);
    pos_jointspace(i, :) = [s1, s2, s3];
end

% --- Joint velocities using central differences
vel_jointspace(2:N-1, :) = (pos_jointspace(3:N, :) - pos_jointspace(1:N-2, :)) / (2*dt);
vel_jointspace(1, :)     = (pos_jointspace(2, :) - pos_jointspace(1, :)) / dt;
vel_jointspace(N, :)     = (pos_jointspace(N, :) - pos_jointspace(N-1, :)) / dt;

% --- Joint accelerations using central differences
acc_jointspace(2:N-1, :) = (pos_jointspace(3:N, :) - 2*pos_jointspace(2:N-1, :) + pos_jointspace(1:N-2, :)) / dt^2;
acc_jointspace(1, :)     = (pos_jointspace(3, :) - 2*pos_jointspace(2, :) + pos_jointspace(1, :)) / dt^2;
acc_jointspace(N, :)     = (pos_jointspace(N, :) - 2*pos_jointspace(N-1, :) + pos_jointspace(N-2, :)) / dt^2;

[pos_taskspace1,t1,v1,a1] = Delta_MoveJ(P1, P2,v_max,a_max);
[pos_taskspace2,t2,v2,a2] = Delta_MoveJ(P2, P3,v_max,a_max);
v2 = [v1 v2];
a2 = [a1 a2];
t2 = [t1 t2+t1(end)];

% pos_jointspace = [pos_taskspace1 pos_taskspace2];

% --- Plot results
figure;
subplot(2,3,1);
plot(t, vel_jointspace(:,1),Linestyle="-",LineWidth=1.3,Color=[0 0 0]);
hold on
plot(t2, v2(1,:), Linestyle="--",LineWidth=1.3,Color=[0 0 0]);
xlabel('Time in s');
ylabel('Joint Velocity in mm/s');
legend('task space', 'join space');
title('Joint 1 Velocities');
grid on;

subplot(2,3,2);
plot(t, vel_jointspace(:,2), Linestyle="-",LineWidth=1.3,Color=[0 0 0]);
hold on
plot(t2, v2(2,:), Linestyle="--",LineWidth=1.3,Color=[0 0 0]);
xlabel('Time in s');
ylabel('Joint Velocity in mm/s');
legend('task space', 'join space');
title('Joint 2 Velocities');
grid on;

subplot(2,3,3);
plot(t, vel_jointspace(:,3),Linestyle="-",LineWidth=1.3,Color=[0 0 0]);
hold on
plot(t2, v2(3,:),Linestyle="--",LineWidth=1.3,Color=[0 0 0]);
xlabel('Time in s');
ylabel('Joint Velocity in mm/s');
legend('task space', 'join space');
title('Joint 3 Velocities');
grid on;

% accelerations
subplot(2,3,4);
plot(t, acc_jointspace(:,1),Linestyle="-",LineWidth=1.3,Color=[0 0 0]);
hold on
plot(t2, a2(1,:),Linestyle="--",LineWidth=1.3,Color=[0 0 0]);
xlabel('Time in s');
ylabel('Joint acc in mm/s^2');
legend('task space', 'join space');
title('Joint 1 Accelerations');
grid on;

subplot(2,3,5);
plot(t, acc_jointspace(:,2),Linestyle="-",LineWidth=1.3,Color=[0 0 0]);
hold on
plot(t2, a2(2,:),Linestyle="--",LineWidth=1.3,Color=[0 0 0]);
xlabel('Time in s');
ylabel('Joint acc in mm/s^2');
legend('task space', 'join space');
title('Joint 2 Accelerations');
grid on;

subplot(2,3,6);
plot(t, acc_jointspace(:,3),Linestyle="-",LineWidth=1.3,Color=[0 0 0]);
hold on
plot(t2, a2(3,:),Linestyle="--",LineWidth=1.3,Color=[0 0 0]);
xlabel('Time in s');
ylabel('Joint acc in mm/s^2');
legend('task space', 'join space');
title('Joint 3 Accelerations');
grid on;

set(gcf, 'color','w')

%%
simin.time = t';
simin.signals.values = pos_taskspace;
