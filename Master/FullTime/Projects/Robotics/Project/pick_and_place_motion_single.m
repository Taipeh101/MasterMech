function [time, pos_jointspace] = pick_and_place_motion_single(pick,drop,joint_max_vel,joint_max_acc,task_max_vel,task_max_acc,task_nom_vel,task_nom_acc)

P_home = [0;0;356.521686];
P_pick = get_target_position(pick);
P_drop = get_target_position(drop);

%% pick object
% approach object
[pos_jointspace1, time1, v1, a1] = Delta_MoveJ(P_home, P_pick + [0 0 -15]', joint_max_vel, joint_max_acc);

% move down
[pos_taskspace2, v2, a2, time2] = Delta_MoveL(P_pick + [0 0 -15]', P_pick,task_nom_vel,task_nom_acc);
pos_jointspace2 = Delta_Task2Joint(pos_taskspace2);

%hold at target for one second
time3 = 0:0.01:1;
pos_jointspace3 = zeros(3,length(time3)) + pos_jointspace2(:,end);

% move up
[pos_taskspace4, v4, a4, time4] = Delta_MoveL(P_pick,P_pick + [0 0 -15]',task_nom_vel,task_nom_acc);
pos_jointspace4 = Delta_Task2Joint(pos_taskspace4);

%% drop object
% approach drop position
[pos_jointspace5, time5, v5, a5] = Delta_MoveJ(P_pick + [0 0 -15]', P_drop + [0 0 -15]', joint_max_vel, joint_max_acc);

% move down
[pos_taskspace6, v6, a6, time6] = Delta_MoveL(P_drop + [0 0 -15]', P_drop,task_nom_vel,task_nom_acc);
pos_jointspace6 = Delta_Task2Joint(pos_taskspace6);

%hold at target for one second
time7 = 0:0.01:1;
pos_jointspace7 = zeros(3,length(time7)) + pos_jointspace6(:,end);

% move up
[pos_taskspace8, v8, a8, time8] = Delta_MoveL(P_drop,P_drop + [0 0 -15]',task_nom_vel,task_nom_acc);
pos_jointspace8 = Delta_Task2Joint(pos_taskspace8);

%% homing
[pos_jointspace9, time9, v9, a9] = Delta_MoveJ(P_drop + [0 0 -15]', P_home, joint_max_vel, joint_max_acc);


% concatanate all
t1 = time1(end);
t2 = t1 + time2(end);    %t1 can be here
t3 = t2 + time3(end);
t4 = t3 + time4(end);
t5 = t4 + time5(end);
t6 = t5 + time6(end);
t7 = t6 + time7(end);
t8 = t7 + time8(end);
pos_jointspace = [pos_jointspace1 pos_jointspace2 pos_jointspace3 pos_jointspace4 pos_jointspace5 pos_jointspace6 pos_jointspace7 pos_jointspace8 pos_jointspace9];
time = [time1 time2+t1 time3+t2 time4+t3 time5+t4 time6+t5 time7+t6 time8+t7 time9+t8];

% pos_jointspace = [pos_jointspace2 pos_jointspace3 pos_jointspace4 pos_jointspace5 pos_jointspace6 pos_jointspace7 pos_jointspace8];
% time = [time2 time3+t2 time4+t3 time5+t4 time6+t5 time7+t6 time8+t7];
end