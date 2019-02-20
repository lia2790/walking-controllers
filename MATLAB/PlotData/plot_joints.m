parts = struct();

parts.torso = {'torso_pitch', 'torso_roll', 'torso_yaw'};
parts.left_shoulder = {'l_shoulder_pitch', 'l_shoulder_roll', 'l_shoulder_yaw', 'l_elbow'};
parts.right_shoulder= {'r_shoulder_pitch', 'r_shoulder_roll', 'r_shoulder_yaw', 'r_elbow'};
parts.left_leg = {'l_hip_pitch', 'l_hip_roll', 'l_hip_yaw', 'l_knee', 'l_ankle_pitch', 'l_ankle_roll'};
parts.right_leg = {'r_hip_pitch', 'r_hip_roll', 'r_hip_yaw', 'r_knee', 'r_ankle_pitch', 'r_ankle_roll'};
                      
type = 'vel';
des = 0;


i = 1;

for part = fieldnames(parts)'
    subplot(2,3,i)
    
    hold on;
    
    label = {};
    
    for joint = parts.(part{:})
        jointValue = joint{:};
        if(des)
            plot(time, eval([jointValue '_des_' type]), time, eval([jointValue '_' type]))
            label = [label, [jointValue '_des_' type], [jointValue '_' type]];            
        else
            plot(time, eval([jointValue '_' type]))
            label = [label, [jointValue '_' type]];
        end
    end
    
    plot_aesthetic([type ' ' part{:}], 'time (s)' , '', '', label{:});
    
    i = i + 1;
end