close all;


subplot(1,3,1)
plot(time, eval('vrp_des_x'), time, eval('zmp_x'))
plot_aesthetic('ZMP - X','Time (s)', 'Positon (m)', ' ', 'Desired' , 'Measured');

subplot(1,3,2)
plot(time, eval('vrp_des_y'), time, eval('zmp_y'))
plot_aesthetic('ZMP - Y','Time (s)', 'Positon (m)', ' ', 'Desired' , 'Measured');

subplot(1,3,3)
plot(time, eval('dcm_des_z'), time, eval('com_z'))
plot_aesthetic('CoM - Z','Time (s)', 'Positon (m)', ' ', 'Desired' , 'Measured');

