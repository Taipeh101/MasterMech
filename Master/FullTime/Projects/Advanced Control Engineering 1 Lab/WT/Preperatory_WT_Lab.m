clear all
clc

% Elias Karner, Leon Haffner
% 15.01.25
%Water Tank

%Task PID control

%% set parameters for system

% given Variables       % [unit]
A_t     = 50*1e-4;      % [m^2]     ... area of tank
A_out   = 25*1e-6;      % [m^2]     ... area of outlet
pl      = 0.05;         % [---]     ... pressure loss
d_w     = 1000  ;       % [kg/m^3]  ... density water
g       = 9.81;         % [m/s^2]   ... gravity constant
h_max   = 0.3;          % [m]       ... height of emergancy outlet
u_max   = 5;           % [V]       ... max pump voltage
K_pump  = 1/60;         % [l/Vs]    ... pump constant

% parameters for symplified model

%a = A_out/A_t*sqrt(2*g/(1+pl));

a_1 = 0.048; % wurden experimentell berechnet
a_2 = 0.0497; % wurden experimentell berechnet

% intitial values and saturation boundaries
h2_0 = 0.0001;
h1_0 = 0.0001;

initial         = [h1_0 h2_0]';

upper_sat       = [h_max h_max]';
lower_sat       = [0 0]';



%% modelling of linearized system

% derived state space of linearized model
% x = [h1 h2]

A = [-a_1*1/(2*sqrt(h1_0)) 0;a_1*1/(2*sqrt(h1_0)) -a_2*1/(2*sqrt(h2_0))];
B = [K_pump/(d_w*A_t); 0];
C = [0 1];
D = 0;

Gs = ss(A,B,C,D);
[Glnum Glden] = ss2tf(A,B,C,D);
G_lin = tf(Glnum,Glden);
%% set controller parameters for system

Kp = 70; %851;       % Proportionalanteil
Ki = 1.4; %108.6;      % Integralanteil
Kd = 55; %1668;       % Differentialanteil

%% trajectory planing with FF

y_T = 0.08;
y_0 = 0.04;

% create time vector
T               = 140;              % Time of simulation
t_0T            = 0:0.01:T;        % Time vector from 0 to T with sample time dT
tau             = t_0T/T;          % Scaling for the prototype function: tau = t/T
t_sim           = 0:0.01:500;      % Time vector from 0 to 2*T with sample time dT for simulation

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

y_h1 = yf(2,:).*yf(2,:)/(a_1)^2+(a_2)^2*yf(1,:)/(a_1)^2;
%y_h1_dot = 2*yf(3,:)/a^2+yf(2,:);
y_h1_dot= 2*(yf(2,:)/a_1+sqrt(yf(1,:))).*(yf(3,:)/a_1+1./(2*sqrt(yf(1,:))).*yf(2,:));

uref=(y_h1_dot + a_1*sqrt(y_h1))*d_w*A_t/K_pump;

uref = [uref uref(end)*ones(1,length(t_sim)-length(tau))];

simin.time = t_sim';
simin.signals.values = [uref;yref]';