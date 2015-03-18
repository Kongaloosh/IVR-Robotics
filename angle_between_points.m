function [theta] = angle_between_points(x_p, y_p, x_r, y_r)
%ANGLE_BETWEEN_POINTS determines the angle between two points.
% given a point robot and a point_path, this defines
% how the robot should be facing relative to north to be
% facing the next point

    w = x_r - x_p;
    h = y_r - y_p;
    
    theta = atan(h/w) / pi * 180;
    
    if (w < 0 || h < 0)
        theta = theta + 180;
        
    elseif (w > 0 && h < 0)
        theta = theta - 180;

    elseif (theta < 0)
        theta = theta + 360;
    end
    theta = mod(theta,360);
    
    % convert to radians because I was silly
    theta = theta * pi/180;

end
