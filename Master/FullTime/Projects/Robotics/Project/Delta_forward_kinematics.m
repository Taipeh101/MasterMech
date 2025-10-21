function [x, y, z] = Delta_forward_kinematics(s1, s2, s3)

    rb = 222.105421; % Base radius
    rm = 42;        % Moving platform radius
    l = 400;        % Length of the robot's legs

    % Define the auxiliary variables
    x1 = 0;
    y1 = rb - s1 * cosd(45) - rm;
    z1 = s1 * sind(45);

    x2 = (rb - s2 * cosd(45) - rm) * cosd(30);
    y2 = -(rb - s2 * cosd(45) - rm) * sind(30);
    z2 = s2 * sind(45);

    x3 = -(rb - s3 * cosd(45) - rm) * cosd(30);
    y3 = -(rb - s3 * cosd(45) - rm) * sind(30);
    z3 = s3 * sind(45);

    % Calculate intermediate variables
    w1 =y1^2 + z1^2;
    w2 = x2^2 + y2^2 + z2^2;
    w3 = x3^2 + y3^2 + z3^2;

    % Intermediate calculations for x and y
    alpha1 = (x3 * (w2 - w1) - x2 * (w3 - w1)) / 2;
    beta1 = (x2 * (z3 - z1) - x3 * (z2 - z1));

    alpha2 = ((y2 - y1) * (w3 - w1) - (y3 - y1) * (w2 - w1)) / 2;
    beta2 = ((y3 - y1) * (z2 - z1) - (y2 - y1) * (z3 - z1));

    d = x3*(y2-y1)-x2*(y3-y1);

    % Solve for z using the quadratic equation
    a = beta2^2 + d^2 + beta1^2;
    b = 2 * (alpha2 * beta2 + alpha1 * beta1 - d^2 * z1 - y1 * d * beta1);
    c = alpha2^2 + alpha1^2 - 2 * y1 * d * alpha1 - d^2 * l^2 + d^2 * w1;

    % Discriminant
    delta = b^2 - 4 * a * c;

    if delta < 0
        error('Singular configuration or no real solution.');
    end

    % Calculate z (select the valid solution)
    z = (-b + sqrt(delta)) / (2 * a);

    % Calculate x and y
    y = (alpha1 + beta1 * z)/d;
    x = (alpha2 + beta2 * z)/d;
end
