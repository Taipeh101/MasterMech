%% Delta 360 Inverse Kinematics Calculation
% This script calculates the inverse kinematics of the Delta 360 manipulator
% Given the task space position (x, y, z), it computes the primary variables (s1, s2, s3)

function [s1, s2, s3] = Delta_inverse_kinematics(x,y,z)
    % Input Parameters:
    % x, y, z: End-effector position
    % rb: Base radius
    % rm: Moving platform radius
    % l: Length of the links

    rb  = 222.105421;
    rm  = 42;
    l   = 400;

    % Universal joint positions on the moving platform
    xm1 = x;
    ym1 = y + rm;
    zm1 = z;
    xm2 = x * cosd(240) + y * sind(240);
    ym2 = y * cosd(240) - x * sind(240) + rm;
    zm2 = z;
    xm3 = x * cosd(120) + y * sind(120);
    ym3 = y * cosd(120) - x * sind(120) + rm;
    zm3 = z;

    % Quadratic coefficients for each leg
    a = 1;
    % Leg 1
    b1 = -2 * zm1 * sind(45) - 2 * rb * cosd(45) + 2 * ym1 * cosd(45);
    c1 = xm1^2 + ym1^2 + zm1^2 + rb^2 - 2 * ym1 * rb - l^2;
    s1 = (-b1 - sqrt(b1^2 - 4 * a * c1)) / (2 * a);

    % Leg 2
    b2 = -2 * zm2 * sind(45) - 2 * rb * cosd(45) + 2 * ym2 * cosd(45);
    c2 = xm2^2 + ym2^2 + zm2^2 + rb^2 - 2 * ym2 * rb - l^2;
    s2 = (-b2 - sqrt(b2^2 - 4 * a * c2)) / (2 * a);

    % Leg 3
    b3 = -2 * zm3 * sind(45) - 2 * rb * cosd(45) + 2 * ym3 * cosd(45);
    c3 = xm3^2 + ym3^2 + zm3^2 + rb^2 - 2 * ym3 * rb - l^2;
    s3 = (-b3 - sqrt(b3^2 - 4 * a * c3)) / (2 * a);
    
    % Display results
    % fprintf('s1 = %.4f mm\n', s1);
    % fprintf('s2 = %.4f mm\n', s2);
    % fprintf('s3 = %.4f mm\n', s3);
end

