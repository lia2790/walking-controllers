close all;

foot_name = 'lf';

subplot(2,3,1)
plot(time, eval([foot_name '_force_des_x']), time, eval([foot_name '_force_x']))
plot_aesthetic('Force - X','Time (s)', 'Force (N)', ' ', 'Desired' , 'Measured');

subplot(2,3,2)
plot(time, eval([foot_name '_force_des_y']), time, eval([foot_name '_force_y']))
plot_aesthetic('Force - Y','Time (s)', 'Force (N)', ' ', 'Desired' , 'Measured');

subplot(2,3,3)
plot(time, eval([foot_name '_force_des_z']), time, eval([foot_name '_force_z']))
plot_aesthetic('Force - Z','Time (s)', 'Force (N)', ' ', 'Desired' , 'Measured');

subplot(2,3,4)
plot(time, eval([foot_name '_torque_des_x']), time, eval([foot_name '_torque_x']))
plot_aesthetic('Moment - X','Time (s)', 'Moment (Nm)', ' ', 'Desired' , 'Measured');

subplot(2,3,5)
plot(time, eval([foot_name '_torque_des_y']), time, eval([foot_name '_torque_y']))
plot_aesthetic('Moment - Y','Time (s)', 'Moment (Nm)', ' ', 'Desired' , 'Measured');

subplot(2,3,6)
plot(time, eval([foot_name '_torque_des_z']), time, eval([foot_name '_torque_z']))
plot_aesthetic('Moment - Z','Time (s)', 'Moment (Nm)', ' ', 'Desired' , 'Measured');