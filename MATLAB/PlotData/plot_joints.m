parts = struct();


parts.torso = {'torso_pitch', 'torso_roll', 'torso_yaw'};
parts.left_arm = {'l_shoulder_pitch', 'l_shoulder_roll', 'l_shoulder_yaw', 'l_elbow'};
parts.right_arm= {'r_shoulder_pitch', 'r_shoulder_roll', 'r_shoulder_yaw', 'r_elbow'};
parts.left_leg = {'l_hip_pitch', 'l_hip_roll', 'l_hip_yaw', 'l_knee', 'l_ankle_pitch', 'l_ankle_roll'};
parts.right_leg = {'r_hip_pitch', 'r_hip_roll', 'r_hip_yaw', 'r_knee', 'r_ankle_pitch', 'r_ankle_roll'};
                      
type = 'trq';
des = 1;

close all

 energy = struct();

for part = fieldnames(parts)'
    figure;
    
    hold on;
    
    
    l = length(parts.(part{:}));
    
    l
    
    cols = 0
    rows = 0
    
    if(~mod(l,3))
        rows = 3;
        cols = l / 3;
    end
    
    if(~mod(l,2))
        rows = 2;
        cols = l / 2;
    end
    
    i = 1;    
    
    rows 
    cols
    

    
   energy.(part{:}) = []
    
    for joint = parts.(part{:})       
    
        label = {};
        
        subplot(rows, cols, i)
        
        jointValue = joint{:};
        if(~strcmp(type, 'pow'))
           if(des)
               plot(time, eval([jointValue '_des_' type]))
               hold on;
               label = [label, ['measured']];
               plotLines;
           else
               plot(time, eval([jointValue '_' type]))
               plotLines;
               label = [label, 'measured'];
           end
        else
            e = eval([jointValue '_trq']) .* eval([jointValue '_vel']);
            energy.(part{:}) = [energy.(part{:}), sum(e(e>0)) * 0.01];
            plot(time, eval([jointValue '_trq']) .* eval([jointValue '_vel']));
            plotLines;
            label = [label, 'measured'];
        end
        str = regexprep(jointValue, '_', ' ');
        plot_aesthetic([str], 'time (s)' , 'Acceleration ($rad/s^2$)', '', label{:}, 'Left stance', 'Right stance', 'Init DS');
        
        i = i + 1;
    end
   
end