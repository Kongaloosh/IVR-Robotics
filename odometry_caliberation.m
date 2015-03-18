function [] = odometry_caliberation()
%ODOMETRY_CALIBERATION Summary of this function goes here
%   Detailed explanation goes here
    x_r = 0;
    y_r = 0;
    phi = 0.5*pi % we start at 0.5 * pi because we presume ourselves to b
                   %facing north
    theta = angle_between_points(x_r,y_r,-4,-4)
    pause(1)
    while(abs(theta-phi) > 0.1)
        wb_robot_step(64);
        disp(phi)
        wb_differential_wheels_set_speed(-1, 1);
        [x_r,y_r,phi] = odometry(-1, 1, x_r, y_r, phi, 0);
        
    end
    wb_differential_wheels_set_speed(0, 0);
    wb_robot_step(64);
end

