clc
clear all
close all

v_max = 50;
a_max = 100;

P1 = [10;30;400];
P2 = [10;0;395];

[pos_jointspace1, time1,v,a] = Delta_MoveJ(P1, P2, v_max, a_max);

plot(time1, pos_jointspace1)

simin.time = time1';
simin.signals.values = pos_jointspace1';