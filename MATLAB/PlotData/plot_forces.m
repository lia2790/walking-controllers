figure
foot_name = 'rf';

foot_name_label = 'Left Foot';
if(strcmp(foot_name, 'rf'))
    foot_name_label = 'Right Foot';
end

roll = eval([foot_name '_roll']);
pitch = eval([foot_name '_pitch']);
yaw = eval([foot_name '_yaw']);

force_x_des = eval([foot_name '_force_des_x']);
force_y_des = eval([foot_name '_force_des_y']);
force_z_des = eval([foot_name '_force_des_z']);
torque_x_des = eval([foot_name '_torque_des_x']);
torque_y_des = eval([foot_name '_torque_des_y']);
torque_z_des = eval([foot_name '_torque_des_z']);

force_x_des_rotated = [];
force_y_des_rotated = [];
force_z_des_rotated = [];
torque_x_des_rotated = [];
torque_y_des_rotated = [];
torque_z_des_rotated = [];



w_iDynTree = iDynTree.Wrench;
%%% Mixed to Body
for i = 1:length(eval([foot_name '_force_x']))
    R = iDynTree.Rotation.RPY(roll(i), pitch(i), yaw(i))
    w_iDynTree.fromMatlab([force_x_des(i), force_y_des(i), force_z_des(i), ...
        torque_x_des(i), torque_y_des(i), torque_z_des(i)]);
    
    w_rotated_idyn = (R.inverse * w_iDynTree);
    w_rotated = w_rotated_idyn.toMatlab;
    force_x_des_rotated = [force_x_des_rotated, w_rotated(1)];
    force_y_des_rotated = [force_y_des_rotated, w_rotated(2)];
    force_z_des_rotated = [force_z_des_rotated, w_rotated(3)];
    torque_x_des_rotated = [torque_x_des_rotated, w_rotated(4)];
    torque_y_des_rotated = [torque_y_des_rotated, w_rotated(5)];
    torque_z_des_rotated = [torque_z_des_rotated, w_rotated(6)];

end

disp ('plotting')

subplot(2,3,1)
plot(time, force_x_des_rotated, time, eval([foot_name '_force_x']))
plotLines;
plot_aesthetic([foot_name_label ' Force - X'],'Time (s)', 'Force (N)', ' ','Desired' , 'Measured', 'Left stance', 'Right stance', 'Init DS');

subplot(2,3,2)
plot(time, force_y_des_rotated, time, eval([foot_name '_force_y']))
plotLines;
plot_aesthetic([foot_name_label ' Force - Y'],'Time (s)', 'Force (N)', ' ', 'Desired' , 'Measured', 'Left stance', 'Right stance', 'Init DS');

subplot(2,3,3)
plot(time, force_z_des_rotated ,time, eval([foot_name '_force_z']))
plotLines;
plot_aesthetic([foot_name_label ' Force - Z'],'Time (s)', 'Force (N)', ' ', 'Desired' , 'Measured', 'Left stance', 'Right stance', 'Init DS');

subplot(2,3,4)
plot(time, torque_x_des_rotated, time, eval([foot_name '_torque_x']))
plotLines;
plot_aesthetic([foot_name_label ' Torque - X'],'Time (s)', 'Moment (Nm)', ' ','Desired' , 'Measured', 'Left stance', 'Right stance', 'Init DS'); 

subplot(2,3,5)
plot(time, torque_y_des_rotated, time, eval([foot_name '_torque_y']))
plotLines;
plot_aesthetic([foot_name_label ' Torque - Y'],'Time (s)', 'Moment (Nm)', ' ', 'Desired' , 'Measured', 'Left stance', 'Right stance', 'Init DS');

subplot(2,3,6)
plot(time, torque_z_des_rotated, time, eval([foot_name '_torque_z']))
plotLines;
plot_aesthetic([foot_name_label ' Torque - Z'],'Time (s)', 'Moment (Nm)', ' ', 'Desired' , 'Measured', 'Left stance', 'Right stance', 'Init DS');