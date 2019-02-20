close all;

foot_name = 'lf';

subplot(2,3,1)
plot(time, eval([foot_name '_des_x']), time, eval([foot_name '_x']))
plot_aesthetic('Position - X','Time (s)', 'Positon (m)', ' ', 'Desired' , 'Measured');

subplot(2,3,2)
plot(time, eval([foot_name '_des_y']), time, eval([foot_name '_y']))
plot_aesthetic('Position - Y','Time (s)', 'Positon (m)', ' ', 'Desired' , 'Measured');

subplot(2,3,3)
plot(time, eval([foot_name '_des_z']), time, eval([foot_name '_z']))
plot_aesthetic('Position - Z','Time (s)', 'Positon (m)', ' ', 'Desired' , 'Measured');

subplot(2,3,4)
plot(time, 180/pi * eval([foot_name '_des_roll']), time, 180/pi * eval([foot_name '_roll']))
plot_aesthetic('Roll','Time (s)', 'Angle (deg)', ' ', 'Desired' , 'Measured');

subplot(2,3,5)
plot(time, 180/pi * eval([foot_name '_des_pitch']), time, 180/pi * eval([foot_name '_pitch']))
plot_aesthetic('Pitch','Time (s)', 'Angle (deg)', ' ', 'Desired' , 'Measured');

subplot(2,3,6)
plot(time, 180/pi * eval([foot_name '_des_yaw']), time, 180/pi * eval([foot_name '_yaw']))
plot_aesthetic('Yaw','Time (s)', 'Angle (deg)', ' ', 'Desired' , 'Measured');