function [] = turn_to_face_point(x_p,y_p)
    x_r = 0;
    y_r = 0;
    phi = 0;
    is_facing = false;
    
    while(not(is_facing))
        wb_robot_step(64);
        theta = angle_between_points(x_p,y_p,x_r,y_r);
%         disp(phi)
%         disp(theta)
        disp('vals')
        disp(abs(phi - theta))
        % if we are further from north than the  
        if (phi - theta > 0)
            % turn left
            disp('left')
            vleft = 3;
            vright = -3;
        % if we are closer
        else
            % turn right
            disp('right')
            vleft = -3;
            vright = 3;
        end
        wb_differential_wheels_set_speed(vleft, vright);
        [x_r,y_r,phi] = odometry(vleft, vright, x_r, y_r, phi, 0);
        if abs(theta - phi) < 2
            is_facing = true;
        end
    end
    wb_differential_wheels_set_speed(0, 0);
end
           
        
        