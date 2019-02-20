close all;

foot_name = 'torso';

subplot(1,3,1)
plot(time, 180/pi * eval([foot_name '_roll_cart_des']), time, 180/pi * eval([foot_name '_roll_cart']))
plot_aesthetic('Roll','Time (s)', 'Angle (deg)', ' ', 'Desired' , 'Measured');

subplot(1,3,2)
plot(time, 180/pi * eval([foot_name '_pitch_cart_des']), time, 180/pi * eval([foot_name '_pitch_cart']))
plot_aesthetic('Pitch','Time (s)', 'Angle (deg)', ' ', 'Desired' , 'Measured');

subplot(1,3,3)
plot(time, 180/pi * eval([foot_name '_yaw_cart_des']), time, 180/pi * eval([foot_name '_yaw_cart']))
plot_aesthetic('Yaw','Time (s)', 'Angle (deg)', ' ', 'Desired' , 'Measured');