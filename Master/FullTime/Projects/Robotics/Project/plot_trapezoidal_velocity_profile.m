function [t,s,v,a] =plot_trapezoidal_velocity_profile(t_acc, t_const, t_dec, v_max , P_start, P_end, k)
% Function to plot trapezoidal velocity, acceleration, and path profiles
% Inputs:
%   t_acc  - Acceleration time [s]
%   t_const - Constant velocity time [s]
%   t_dec  - Deceleration time [s]
%   v_max  - Maximum velocity [m/s]

a_max = v_max / t_acc;

% Time vectors for each phase
dt = 0.00001;
t1 = 0:dt:t_acc;                    % Acceleration phase
t2 = t1(end):dt:t_const;            % Constant velocity phase
t3 = t2(end):dt:t_dec;              % Deceleration phase

% % Time vectors for each phase
% t1 = linspace(0, t_acc, 100);       % Acceleration phase
% t2 = linspace(t_acc,  t_const, 100);% Constant velocity phase
% t3 = linspace(t_const, t_dec, 100); % Deceleration phase


% Velocity profiles
v1 = a_max * k * t1;                   % Acceleration
v2 = v_max * k * ones(size(t2));       % Constant velocity
v3 = v_max * k - a_max * k * (t3 - t2(end)); % Deceleration

% Acceleration profiles
a1 = a_max * k * ones(size(t1));       % Constant acceleration
a2 = zeros(size(t2));                  % Zero acceleration
a3 = -a_max * k * ones(size(t3));      % Negative acceleration

% Path profiles (displacement)
s1 = P_start + 0.5 * a_max * k * t1.^2;                                      % During acceleration
s2 = s1(end) + v_max * k * (t2 - t2(1));                                     % During constant velocity
s3 = s2(end) + v_max * k * (t3 - t3(1)) - 0.5 * a_max * k * (t3 - t3(1)).^2; % During deceleration

% Combine time and profiles
t = [t1, t2, t3];
v = [v1, v2, v3];
a = [a1, a2, a3];
s = [s1, s2, s3];

% Return only values at 10 ms intervals
sample_indices = find(mod(t, 0.01) < dt);  % tolerance due to floating-point precision
t = t(sample_indices);
s = s(sample_indices);
s(end) = P_end;
v = v(sample_indices);
a = a(sample_indices);

% Plot path profile

subplot(3,1,1);
plot(t, s, 'LineWidth', 2);
grid on;
title('Path Profile');
xlabel('Time [s]');
ylabel('Displacement [m]');
hold on

% Plot velocity profile
subplot(3,1,2);
plot(t, v, 'LineWidth', 2);
grid on;
title('Trapezoidal Velocity Profile');
xlabel('Time [s]');
ylabel('Velocity [m/s]');
hold on

% Plot acceleration profile
subplot(3,1,3);
plot(t, a, 'LineWidth', 2);
grid on;
title('Acceleration Profile');
xlabel('Time [s]');
ylabel('Acceleration [m/s^2]');
hold on

end