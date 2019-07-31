parts = struct();


parts.torso = {'torso_pitch', 'torso_roll', 'torso_yaw'};
parts.left_arm = {'l_shoulder_pitch', 'l_shoulder_roll', 'l_shoulder_yaw', 'l_elbow'};
parts.right_arm= {'r_shoulder_pitch', 'r_shoulder_roll', 'r_shoulder_yaw', 'r_elbow'};
parts.left_leg = {'l_hip_pitch', 'l_hip_roll', 'l_hip_yaw', 'l_knee', 'l_ankle_pitch', 'l_ankle_roll'};
parts.right_leg = {'r_hip_pitch', 'r_hip_roll', 'r_hip_yaw', 'r_knee', 'r_ankle_pitch', 'r_ankle_roll'};
                      
type = 'trq';
des = 1;

close all


for part = fieldnames(parts)'
    figure;
    
    hold on;
    
    
    l = length(parts.(part{:}));
    
    l
    
    cols = 0
    rows = 0
    
    if(~mod(l,3))
        cols = 3;
        rows = l / 3;
    end
    
    if(~mod(l,2))
        cols = 2;
        rows = l / 2;
    end
    
    i = 1;    
    
    rows 
    cols
    
    for joint = parts.(part{:})       
    
        label = {};
        
        subplot(rows, cols, i)
        
        jointValue = joint{:};
        if(des)
            plot(time, eval([jointValue '_des_' type]), time, eval([jointValue '_' type]))
            hold on;
            label = [label, ['desired'], ['measured']]; 
            plotLines;
        else
            plot(time, eval([jointValue '_' type]))
            label = [label, [jointValue '_' type]];
        end
        str = regexprep(jointValue, '_', ' ');
   
        plot_aesthetic([str], 'time (s)' , 'torque (Nm)', '', label{:}, 'Init SS', 'Init DS');
        
        i = i + 1;
    end
    
   
    
   
end