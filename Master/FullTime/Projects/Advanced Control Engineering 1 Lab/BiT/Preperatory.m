clear all
clc

% Elias Karner, Leon Haffner
% 27.11.24
%Ball in Tube 

%Task PID control

%% set parameters for system

% given Variables       % [unit]

A_b     = 13*1e-4;      % [m^2]     ... area ball
m       = 2.7*1e-3;     % [kg]      ... mass of ball
A_t     = 15*1e-4;      % [m^2]     ... area tube
k_l     = 2*1e-5;       % [m^3]     ... drag coefficient
k_v     = 1.27*1e-4;    % [kg/m]    ... air flow coefficient
tau_w   = 1.6;          % [s]       ... fan time constant
k_w     = 28;           % [1/(s*V)] ... fan gain
rho_l   = 1.225;        % [kg/m^3]  ... density of air
g       = 9.81;         % [m/s^2]   ... gravity constant
h_max   = 0.4;          % [m]       ... length of tube
u_max   = 5;            % [V]       ... max fan voltage
A_gap   = A_t-A_b;      % [m^2]     ... air gap

% parameters for symplified model

a3 = g;
a2 = (A_b*sqrt(k_l))/(sqrt(m)*A_gap);
a1 = (k_v*sqrt(k_l))/(sqrt(m)*A_gap);


% intitial values and saturation boundaries
w0 = sqrt(a3)/a1;   % this is the rotational speed right before the ball is lifted up    

initial         = [0.15 0 w0]';

upper_sat       = [0.35 100 200]';
lower_sat       = [0 -100 0]';


% set the desired hight of the ball
y_0             = initial(1);
y_T             = 0.3;             % [m]

% create time vector
T               = 1;              % Time of simulation
t_0T            = 0:0.01:T;        % Time vector from 0 to T with sample time dT
tau             = t_0T/T;          % Scaling for the prototype function: tau = t/T
t_sim           = 0:0.01:100;      % Time vector from 0 to 2*T with sample time dT for simulation
%% modelling of linearized system

% derived state space of linearized model
% x = [d_z d_z_dot w]

A = [0 1 0; 0 -2*a2*sqrt(a3) 2*a1*sqrt(a3); 0 0 -1/tau_w];
B = [0; 0; k_w/tau_w];
C = [1 0 0];
D = 0;

Gs = ss(A,B,C,D);
[Glnum Glden] = ss2tf(A,B,C,D);
G_lin = tf(Glnum,Glden);
%% set controller parameters for system

Kp = 1.079;       % Proportionalanteil
Ki = 0.23;    % Integralanteil
Kd = 1.118;      % Differentialanteil

%% trajectory planing with FF

n = order(G_lin);
phi = zeros(n+1,2*n+2); % Definition of the matrix size: n x 2*n+1

% Term within the sum on the correct column of the row-vector

for k = 0:n
    phi(1,n+1-k) = nchoosek(n,k)*(-1)^k/(n+k+1);
end

% Add factor (which is multiplied by the sum) to all coefficients
phi = factorial(2*n+1)/factorial(n)^2*phi;

for j = 1:n
phi(j+1,j+1:end) = 1/T*polyder(phi(j,j:end));
end

phi4 = zeros(n+1,length(tau));

for i = 1:n+1
    phi4(i,:) = polyval(phi(i,:),tau);
end


yf = (y_T-y_0)*phi4; % add the scaling factor to the prototype-function
yf(1,:) = y_0+yf(1,:); % add the start value for the non-derived function

yref = yf(1,:);


yref = [yref yref(end)*ones(1,length(t_sim)-length(tau))];

omega=(sqrt(yf(3,:)+a3)+a2*yf(2,:))/a1;
omega_dot=1/2*1./sqrt(yf(3,:)+a3).*yf(4,:)./a1+a2/a1.*yf(3,:);
uref=(tau_w*omega_dot+omega)/k_w;

uref = [uref uref(end)*ones(1,length(t_sim)-length(tau))];

simin.time = t_sim';
simin.signals.values = [uref;yref]';