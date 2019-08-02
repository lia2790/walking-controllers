figure

subplot(1,3,1)
plot(time, eval('com_des_x'), time, eval('com_x'))
plotLines;
plot_aesthetic('CoM - X','Time (s)', 'Positon (m)', ' ', 'Desired' , 'Measured', 'Init SS', 'Init DS');

subplot(1,3,2)
plot(time, eval('com_des_y'), time, eval('com_y'))
plotLines;

plot_aesthetic('CoM - Y','Time (s)', 'Positon (m)', ' ', 'Desired' , 'Measured', 'Init SS', 'Init DS');


subplot(1,3,3)

plot(time, eval('dcm_des_z'), time, eval('com_z'))
plotLines;
plot_aesthetic('CoM - Z','Time (s)', 'Positon (m)', ' ', 'Desired' , 'Measured', 'Init SS', 'Init DS');

