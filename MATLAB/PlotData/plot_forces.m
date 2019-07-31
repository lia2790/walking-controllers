close all;

foot_name = 'rf';

subplot(2,3,1)
plot(time, eval([foot_name '_force_des_x']), time, eval([foot_name '_force_x']))
plotLines;
plot_aesthetic('Right Foot Force - X','Time (s)', 'Force (N)', ' ', 'Desired' ,  'Measured', 'Init SS', 'Init DS');

subplot(2,3,2)
plot(time, eval([foot_name '_force_des_y']), time, eval([foot_name '_force_y']))
plotLines;
plot_aesthetic('Right Foot Force - Y','Time (s)', 'Force (N)', ' ', 'Desired' ,  'Measured', 'Init SS', 'Init DS');

subplot(2,3,3)
plot(time, eval([foot_name '_force_des_z']), time, eval([foot_name '_force_z']))
plotLines;
plot_aesthetic('Right Foot Force - Z','Time (s)', 'Force (N)', ' ', 'Desired' ,  'Measured', 'Init SS', 'Init DS');

subplot(2,3,4)
plot(time, eval([foot_name '_torque_des_x']), time, eval([foot_name '_torque_x']))
plotLines;
plot_aesthetic('Right Foot Moment - X','Time (s)', 'Moment (Nm)', ' ', 'Desired' ,  'Measured', 'Init SS', 'Init DS');

subplot(2,3,5)
plot(time, eval([foot_name '_torque_des_y']), time, eval([foot_name '_torque_y']))
plotLines;
plot_aesthetic('Right Foot Moment - Y','Time (s)', 'Moment (Nm)', ' ', 'Desired' ,  'Measured', 'Init SS', 'Init DS');

subplot(2,3,6)
plot(time, eval([foot_name '_torque_des_z']), time, eval([foot_name '_torque_z']))
plotLines;
plot_aesthetic('Right Foot Moment - Z','Time (s)', 'Moment (Nm)', ' ', 'Desired' ,  'Measured', 'Init SS', 'Init DS');