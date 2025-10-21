function [pos_jointspace, t,v,a] = Delta_MoveJ(P1, P2, v_max, a_max)

[s1, s2, s3] = Delta_inverse_kinematics(P1(1),P1(2),P1(3));
P1_s = [s1 s2 s3];
[s1, s2, s3] = Delta_inverse_kinematics(P2(1),P2(2),P2(3));
P2_s = [s1 s2 s3];

Diff = P2_s-P1_s;
t1 = [v_max/a_max v_max/a_max v_max/a_max];

sign_array = sign(Diff);
Diff = abs(Diff);

t2 = Diff./v_max;
t3 = t2 + t1;

distance_acc = 0.5 * a_max * t1.^2;

if(Diff > 2 * distance_acc)
    index = find(t3==max(t3));
    v_new = Diff./t2(index);
    a_new = v_new./t1(index);

    % create time series data
    [t1m,s1m,v1,a1] = plot_trapezoidal_velocity_profile(t1(index),t2(index),t3(index),v_new(1),P1_s(1),P2_s(1),sign_array(1));
    [t2m,s2m,v2,a2] = plot_trapezoidal_velocity_profile(t1(index),t2(index),t3(index),v_new(2),P1_s(2),P2_s(2),sign_array(2));
    [t3m,s3m,v3,a3] = plot_trapezoidal_velocity_profile(t1(index),t2(index),t3(index),v_new(3),P1_s(3),P2_s(3),sign_array(3));
    pos_jointspace = [s1m; s2m; s3m];
    v = [v1; v2; v3];
    a = [a1; a2; a3];
    
    t = t1m;

else    % shape is triangular
    % Triangular profile (no constant velocity phase)
    t1(:) = sqrt(max(Diff) / a_max);
    t2 = t1;
    t3 = t1.*2;
    
    dt = 0.00001;              % time step
    t = 0:dt:t3;            % time vector

    a_new = (Diff./t1.^2).*sign_array;

    % Initialize profiles
    pos_jointspace = zeros(3,length(t));    % position
    v = zeros(3,length(t));                  % velocity
    a = zeros(3,length(t));                  % acceleration

    for i = 1:length(t)
        ti = t(i);
        if ti < t1(1)
            s(:,i) = 0.5 * a_new * ti^2;
            v(:,i) = a_new * ti;
            a(:,i) = a_new;
       
        elseif ti <= t3
            dt_dec = ti - t1(1);
            v_peak = a_new*t1(1);
            s(:,i) = 0.5 * a_new * t1(1)^2 + v_peak * dt_dec - 0.5 * a_new * dt_dec(1)^2;
            v(:,i) = v_peak - a_new * dt_dec;
            a(:,i) = -a_new;
        end
    end
    % Return only values at 10 ms intervals
    sample_indices = find(mod(t, 0.01) < dt);  % tolerance due to floating-point precision
    t = t(sample_indices);
    s = s(:,sample_indices) + P1_s';
    s(:,end) = P2_s;
    v = v(:,sample_indices);
    a = a(:,sample_indices);
    pos_jointspace = s;
end



end