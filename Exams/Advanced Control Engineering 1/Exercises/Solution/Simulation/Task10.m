clear all
clc

T = 1;  % step time
t_sim = 0:0.01:10;
t = 0:0.01:T;

G = tf([2],[1 1 0]);

uref = 15 * t.^4 + 30 * t.^3 - 75*t.^2 + 30*t;
yref = 6*t.^5 - 15*t.^4 + 10 *t.^3;

uref = [uref uref(end)*ones(1,length(t_sim)-length(t))];
yref = [yref yref(end)*ones(1,length(t_sim)-length(t))];

simin.time = t_sim';
simin.signals.values = [uref;yref]';
