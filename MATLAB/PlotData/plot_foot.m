figure
foot_name = 'rf';

subplot(2,3,1)
plot(time, eval([foot_name '_des_x']), time, eval([foot_name '_x']))
plotLines;
plot_aesthetic('Right Foot - X','Time (s)', 'Positon (m)', ' ', 'Desired' , 'Measured', 'Init SS', 'Init DS');

subplot(2,3,2)
plot(time, eval([foot_name '_des_y']), time, eval([foot_name '_y']))
plotLines;
plot_aesthetic('Right Foot - Y','Time (s)', 'Positon (m)', ' ', 'Desired' , 'Measured','Init SS', 'Init DS');


subplot(2,3,3)
plot(time, eval([foot_name '_des_z']), time, eval([foot_name '_z']))
plotLines;
plot_aesthetic('Right Foot - Z','Time (s)', 'Positon (m)', ' ', 'Desired' , 'Measured','Init SS', 'Init DS');


subplot(2,3,4)
plot(time, 180/pi * eval([foot_name '_des_roll']), time, 180/pi * eval([foot_name '_roll']))
plotLines;
plot_aesthetic('Right Foot - Roll','Time (s)', 'Angle (deg)', ' ', 'Desired' , 'Measured','Init SS', 'Init DS');


subplot(2,3,5)
plot(time, 180/pi * eval([foot_name '_des_pitch']), time, 180/pi * eval([foot_name '_pitch']))
plotLines;
plot_aesthetic('Right Foot - Pitch','Time (s)', 'Angle (deg)', ' ', 'Desired' , 'Measured','Init SS', 'Init DS');


subplot(2,3,6)
plot(time, 180/pi * eval([foot_name '_des_yaw']), time, 180/pi * eval([foot_name '_yaw']))
plotLines;
plot_aesthetic('Right Foot - Yaw','Time (s)', 'Angle (deg)', ' ', 'Desired' , 'Measured','Init SS', 'Init DS');
