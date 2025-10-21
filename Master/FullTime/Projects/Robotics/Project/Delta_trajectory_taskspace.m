clc
clear all
close all



% Start and end points in task space
P1 = [0; 0; 356.521686];
P2 = [-25; -75; 400];
P3 = [30; 40;400];

[pos_taskspace1,v1,a1, time1] = Delta_MoveL(P1, P2);
[pos_taskspace2,v2,a2, time2] = Delta_MoveL(P2, P3);
% [pos_taskspace3,v3,a3, time3] = Delta_MoveL(P3, P1);

pos_taskspace = [pos_taskspace1 pos_taskspace2];
v = [v1 v2];
a = [a1 a2];
time = [time1 time2+time1(end)];

figure;
subplot(3,1,1);
plot(time, pos_taskspace);
ylabel('Position s(t) [mm]');
grid on;
title('Trapezoidal Trajectory Profiles');

subplot(3,1,2);
plot(time, v, 'r', 'LineWidth', 1.5);
ylabel('Velocity v(t) [mm/s]');
grid on;

subplot(3,1,3);
plot(time, a, 'g', 'LineWidth', 1.5);
ylabel('Acceleration a(t) [mm/s^2]');
xlabel('Time [s]');
grid on;


% Simulink-compatible structure
simin.time = time';
simin.signals.values = pos_taskspace';