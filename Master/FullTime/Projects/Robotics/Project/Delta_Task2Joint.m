function [pos_jointspace] = Delta_Task2Joint(pos_taskspace)
% --- Inverse kinematics: compute joint positions
N = size(pos_taskspace, 2);
for i = 1:N
    x = pos_taskspace(1, i);
    y = pos_taskspace(2, i);
    z = pos_taskspace(3, i);
    [s1, s2, s3] = Delta_inverse_kinematics(x, y, z);
    pos_jointspace(:, i) = [s1, s2, s3];
end
end