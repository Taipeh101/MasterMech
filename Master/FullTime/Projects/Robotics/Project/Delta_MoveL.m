function [pos_taskspace, v, a, t] = Delta_MoveL(P1,P2,v_max,a_max)

% Calculate distance and trajectory shape
distance = norm(P2 - P1);
t1 = v_max / a_max;
distance_acc = 0.5 * a_max * t1^2;

if distance < 2 * distance_acc
    % Triangular profile (no constant velocity phase)
    t1 = sqrt(distance / a_max);
    t2 = t1;
    t3 = t1*2;
else
    % Trapezoidal profile
    distance_flat = distance - 2 * distance_acc;
    t2 = distance_flat / v_max;
    t3 = 2 * t1 + t2;       % total time
end

dt = 0.01;              % time step
t = 0:dt:t3;            % time vector

% Initialize profiles
s = zeros(size(t));     % position
v = zeros(size(t));     % velocity
a = zeros(size(t));     % acceleration

if distance > 2 * distance_acc
    % Trapezoidal profile
    for i = 1:length(t)
        ti = t(i);
        if ti < t1
            s(i) = 0.5 * a_max * ti^2;
            v(i) = a_max * ti;
            a(i) = a_max;
        elseif ti < (t1 + t2)
            s(i) = distance_acc + v_max * (ti - t1);
            v(i) = v_max;
            a(i) = 0;
        elseif ti <= t3
            dt_dec = ti - (t1 + t2);
            s(i) = distance_acc + v_max * t2 + v_max * dt_dec - 0.5 * a_max * dt_dec^2;
            v(i) = v_max - a_max * dt_dec;
            a(i) = -a_max;
        end
    end
else
    % Triangular profile
    for i = 1:length(t)
        ti = t(i);
        if ti < t1
            s(i) = 0.5 * a_max * ti^2;
            v(i) = a_max * ti;
            a(i) = a_max;
       
        elseif ti <= t3
            dt_dec = ti - t1;
            v_peak = a_max * t1;
            s(i) = 0.5 * a_max * t1^2 + v_peak * dt_dec - 0.5 * a_max * dt_dec^2;
            v(i) = v_peak - a_max * dt_dec;
            a(i) = -a_max;
        end
    end
end


% Generate task space position from scalar path s(t)
direction = (P2 - P1) / distance;  % unit direction
pos_taskspace = (P1' + s' * direction')';  % Nx3 matrix

pos_taskspace(:,end) = P2;

figure;
subplot(3,1,1);
plot(t, s, 'b', 'LineWidth', 1.5);
ylabel('Position s(t) [mm]');
grid on;
title('Trapezoidal Trajectory Profiles');

subplot(3,1,2);
plot(t, v, 'r', 'LineWidth', 1.5);
ylabel('Velocity v(t) [mm/s]');
grid on;

subplot(3,1,3);
plot(t, a, 'g', 'LineWidth', 1.5);
ylabel('Acceleration a(t) [mm/s^2]');
xlabel('Time [s]');
grid on;

end