clc;
data = readmatrix('../code/fan_charac/datasets/v_look_up_table.csv');


% idx = find(data(:,2) > 3)
% 
% duty = data(1:idx,1);
% vel = data(1:idx,2);

duty = data(1:115,1);
vel = data(1:115,2);

% scatter(duty,vel)
cftool(vel, duty)

