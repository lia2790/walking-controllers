close all;

prefix = 'dcm';


subplot(1,3,1)
plot(time, eval([prefix '_des_x']), time, eval([prefix '_x']), time, ds)
plot_aesthetic('Position - X','Time (s)', 'Positon (m)', ' ', 'Desired' , 'Measured');

subplot(1,3,2)
plot(time, eval([prefix '_des_y']), time, eval([prefix '_y']))
plot_aesthetic('Position - Y','Time (s)', 'Positon (m)', ' ', 'Desired' , 'Measured');

subplot(1,3,3)
plot(time, eval([prefix '_des_z']), time, eval([prefix '_z']))
plot_aesthetic('Position - Z','Time (s)', 'Positon (m)', ' ', 'Desired' , 'Measured');

