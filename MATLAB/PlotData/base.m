close all;

foot_name = 'base';

subplot(2,3,1)
plot(time, eval([foot_name '_debug_x']), time, eval([foot_name '_x']))
plot_aesthetic('Position - X','Time (s)', ' m ', ' ', 'Debug' , 'Standard');

subplot(2,3,2)
plot(time, eval([foot_name '_debug_y']), time, eval([foot_name '_y']))
plot_aesthetic('Position - Y','Time (s)', 'm', ' ', 'Debug' , 'Standard');

subplot(2,3,3)
plot(time, eval([foot_name '_debug_z']), time, eval([foot_name '_z']))
plot_aesthetic('Position - Z','Time (s)', 'm', ' ', 'Debug' , 'Standard');

subplot(2,3,4)
plot(time, eval([foot_name '_debug_dx']), time, eval([foot_name '_dx']))
plot_aesthetic('Velocity - X','Time (s)', 'm/s', ' ', 'Debug' , 'Standard');

subplot(2,3,5)
plot(time, eval([foot_name '_debug_dy']), time, eval([foot_name '_dy']))
plot_aesthetic('Velocity - Y','Time (s)', 'm/s', ' ', 'Debug' , 'Standard');

subplot(2,3,6)
plot(time, eval([foot_name '_debug_dz']), time, eval([foot_name '_dz']))
plot_aesthetic('Velocity - Z','Time (s)', 'm/s', ' ', 'Debug' , 'Standard');

figure;

subplot(2,3,1)
plot(time, 180 / pi * eval([foot_name '_debug_roll']), time, 180 / pi *  eval([foot_name '_roll']))
plot_aesthetic('Roll ','Time (s)', 'deg', ' ', 'Debug' , 'Standard');

subplot(2,3,2)
plot(time, 180 / pi * eval([foot_name '_debug_pitch']), time, 180 / pi *  eval([foot_name '_pitch']))
plot_aesthetic('Pitch','Time (s)', 'deg', ' ', 'Debug' , 'Standard');

subplot(2,3,3)
plot(time, 180 / pi * eval([foot_name '_debug_yaw']), time, 180 / pi *  eval([foot_name '_yaw']))
plot_aesthetic('yaw','Time (s)', 'deg', ' ', 'Debug' , 'Standard');

subplot(2,3,4)
plot(time, 180 / pi * eval([foot_name '_debug_omega_x']), time, 180 / pi *  eval([foot_name '_omega_x']))
plot_aesthetic('Omega x','Time (s)', 'deg/s', ' ', 'Debug' , 'Standard');

subplot(2,3,5)
plot(time, 180 / pi * eval([foot_name '_debug_omega_y']), time, 180 / pi *  eval([foot_name '_omega_y']))
plot_aesthetic('Omega y','Time (s)', 'deg/s', ' ', 'Debug' , 'Standard');

subplot(2,3,6)
plot(time, 180 / pi * eval([foot_name '_debug_omega_z']), time, 180 / pi *  eval([foot_name '_omega_z']))
plot_aesthetic('Omega z','Time (s)', 'deg/s', ' ', 'Debug' , 'Standard');