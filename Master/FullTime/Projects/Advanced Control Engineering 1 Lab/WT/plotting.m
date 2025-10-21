clc
close all

set = 26;
row = 8;

y1 = data{set}{row}.Values.Data;
y1 = (y1.*1000); % * 0.7
t = data{set}{row}.Values.Time;

y_ref = data{set}{1}.Values.Data;
y_ref = y_ref.*1000;

% y_sim = sim{6}.Values.Data * 1000;
% t_sim = sim{6}.Values.Time;


plot(t, y_ref, 'LineStyle','-','LineWidth',1.5,'Color',[0 0 0])
hold on
plot(t, y1, 'LineStyle','-','LineWidth',1.5,'Color',[0.5 0.5 0.5])
% plot(t_sim, y_sim, 'LineStyle',':','LineWidth',1.5,'Color',[0 0 0])

legend('trajectory','h_{2real}','Location','southeast')
x1 = [0 50 270];
x2 = [0 20 120];
plot_properties(x1,x2)