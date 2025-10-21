% Crane system

% Authors: ...
% Date. 10.06.2025

%% clean up
clear all
clc
close all
%% Initzialisation

% syms zb zc phi zbd zcd phid
% syms mc mb ml kb kc db dc g l F s
if(1)

mb          = 20;    % base mass in                                 [kg]
mc          = 2;     % cart mass in                                 [kg]         
ml          = 1;     % load mass in                                 [kg] 

kb          = 2000;  % spring constant of the base                  [N/m]
kc          = 0;     % NOT yet considered in the model  

db          = 12.16; % damping of the base                          [N/m/s]
dc          = 10;     % damping of the cart to the base (friction)  [N/m/s] 

l           = 0.25;     % length of the rope from cart to load      [m]
g           = 9.81;  % gravitaion constant                          [kg/m/s^2]

% Basisparameter
param.mb  = 20;      % base mass [kg]
param.mc  = 2;       % cart mass [kg]
param.ml  = 1;       % load mass [kg]

% Feder und Dämpfung
param.kb  = 2000;    % spring constant of the base [N/m]
param.kc  = 0;       % not used yet

param.db  = 12.16;   % base damping [N/m/s]
param.dc  = 10;      % cart damping relative to base [N/m/s]

% Geometrie und Gravitation
param.l   = 0.25;    % rope length [m]
param.g   = 9.81;    % gravity [m/s^2]

param.Q=diag([1e-8 1e-12 1e-12 1e-12 1e-6 1e-12]);
param.R=diag([1000 1000]);


param.Ts=1e-4;% Sampling time in s

% additional Parameters for the real system
param.mb_real = 16; %kg
param.db_real = 12.16; % N/m/s



% PID control parameters
Kp = 12;
Ki = 10;
Kd = 0.8;

% state variables
zc          = 0;
dzc         = 0;
zb          = 0;
dzb         = 0;
phi         = 0;
dphi        = 0;

zc_max      = 2;     % max length ot the linear cart drive
end
%% State Space of linear model

% x = [zc dzc zb dzb phi dphi]

A=[0 1 0 0 0 0;
      0 -dc/mc 0  dc/mc ml*g/mc 0;
      0 0 0 1 0 0;
      0 dc/mb -kb/mb -(db+dc)/mb 0 0;
      0 0 0 0 0 1;
      0 dc/(l*mc) 0 -dc/(l*mc) -g*(mc+ml)/(l*mc) 0];

B=[0;1/mc;0;-1/mb;0;-1/(l*mc)];

C=[1 0 0 0 l 0];    % absolute position of the load [zc + l*sin(phi)]
D=[0];

system_l = ss(A,B,C,D);
[num,denum] =ss2tf(A,B,C,D)
G = tf(num,denum);

n = order(G); % n-times continuously differentiable function phi_n (n is also equal to the system order)
dT = param.Ts;


%% LQR

% Q and R for smaller steady state error than PID
% Bryson_rule 
Q_11 = 1/0.001^2;       % max 0.001     [m]
Q_22 = 1/0.05^2;        % max 0.05      [m/s^2]
Q_33 = 1/0.1^2;         % max 0.1       [m]
Q_44 = 1/0.2^2;         % max 0.2       [m/s^2]
Q_55 = 1/0.01^2;        % max 0.01      [rad]
Q_66 = 1/0.5^2;         % max 0.5       [rad/s]     

Q_LQR = [Q_11 0 0 0 0 0
         0 Q_22 0 0 0 0
         0 0 Q_33 0 0 0
         0 0 0 Q_44 0 0
         0 0 0 0 Q_55 0
         0 0 0 0 0 Q_66];
R_LQR = 0.01;       % 0.01

L_gain = lqr(A,B,Q_LQR,R_LQR);

poles = eig(A)-5;                           % set polse for the observer
LD = acker(system_l.A',system_l.C',poles);