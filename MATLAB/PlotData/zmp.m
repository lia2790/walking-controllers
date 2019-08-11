close all;



subplot(1,2,1)
plot(time, eval('vrp_des_x'), time, eval('zmp_x'))
plotLines;
plot_aesthetic('ZMP - X','Time (s)', 'Positon (m)', ' ', 'Desired' , 'Measured', 'Left stance', 'Right stance', 'Init DS');


subplot(1,2,2)
plot(time, eval('vrp_des_y'), time, eval('zmp_y'))
plotLines;
plot_aesthetic('ZMP - Y','Time (s)', 'Positon (m)', ' ', 'Desired' , 'Measured', 'Left stance', 'Right stance', 'Init DS');



foot_offset = [0.063 0.041];
footSize = [0.19, 0.09];


h = figure('Renderer', 'painters', 'Position', [10 10 900 600], 'color','w');
hold on;
plotLeft = 1;
xlim([-0.2 0.6])
ylim([-0.2 0.2])

daspect([1 1 1])

rectangle('Position',[lf_des_x(initDS(1)) - foot_offset(1),...
    lf_des_y(initDS(1)) - foot_offset(2),footSize(1),footSize(2)],...
    'FaceColor',[253 246 227 256/2]/256);

rectangle('Position',[rf_des_x(initDS(1)) - foot_offset(1),...
    rf_des_y(initDS(1)) - foot_offset(2),footSize(1),footSize(2)],...
    'FaceColor',[253 246 227 256/2]/256);



rectangle('Position',[lf_x(initDS(1)) - foot_offset(1),...
    lf_y(initDS(1)) - foot_offset(2),footSize(1),footSize(2)],...
    'FaceColor',[0.4660, 0.6740, 0.1880 0.5]);

rectangle('Position',[rf_x(initDS(1)) - foot_offset(1),...
    rf_y(initDS(1)) - foot_offset(2),footSize(1),footSize(2)],...
    'FaceColor',[0.4660, 0.6740, 0.1880 0.5]);

scatter_des = scatter(vrp_des_x(1), vrp_des_y(1) , 'r');
scatter_meas = scatter(zmp_x(1), zmp_y(1), 'b');

plot_aesthetic('ZMP - X \& Y','x (m)', 'y (m)', ' ', 'Desired' , 'Measured', 'Left stance', 'Right stance', 'Init DS');

filename = 'testAnimated.gif';

gif('myfile_zmp.gif','DelayTime',0.1,'LoopCount',5,'frame',gcf)


    frames = 1
    
for i=2:length(time)
    
    if sum(i == initDS)
        if(plotLeft)
            rectangle('Position',[lf_des_x(i) - foot_offset(1),...
                lf_des_y(i) - foot_offset(2),footSize(1),footSize(2)],...
                'FaceColor',[253 246 227 256/2]/256);
            
            rectangle('Position',[lf_x(i) - foot_offset(1),...
                lf_y(i) - foot_offset(2),footSize(1),footSize(2)],...
                'FaceColor',[0.4660, 0.6740, 0.1880 0.5]);
            
            plotLeft = 0
        else
            rectangle('Position',[rf_des_x(i) - foot_offset(1),...
                rf_des_y(i) - foot_offset(2),footSize(1),footSize(2)],...
                'FaceColor',[253 246 227 256/2]/256);
            
            rectangle('Position',[rf_x(i) - foot_offset(1),...
                rf_y(i) - foot_offset(2),footSize(1),footSize(2)],...
                'FaceColor',[0.4660, 0.6740, 0.1880 0.5]);
            
            plotLeft = 1
        end
        
        uistack(scatter_des,'top')
        uistack(scatter_meas,'top')
    end
    i
    
    scatter_des.XData = [ scatter_des.XData,  vrp_des_x(i)];
    scatter_des.YData = [ scatter_des.YData,  vrp_des_y(i)];
    
    scatter_meas.XData = [ scatter_meas.XData,  zmp_x(i)];
    scatter_meas.YData = [ scatter_meas.YData,  zmp_y(i)];
    
        
    frames = frames + 1;
    
    if(frames  == 10)
        gif
        frames = 1
    end
    
end






