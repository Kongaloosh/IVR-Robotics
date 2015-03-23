% Very simple MATLAB controller for Webots
% File: matlabcontrol.m
% Date: 21/09/2011          
% Description: This controller will open MATLAB and let you input
%              commands from the keyboard
% Author: Simon Smith (artificialsimon@ed.ac.uk)        
% Modifications: 
% Use: command to set motor speed 1 to left motor, -1 to right motor
%       wb_differential_wheels_set_speed(1, -1)
% After sending a setting command, the controller have to resume by
% sending "return" in the keyboard input
% This will allow a sensor value update as well

TIME_STEP = 64;
N = 8;

% get and enable distance sensors
for i=1:N
  ps(i) = wb_robot_get_device(['ds' int2str(i-1)]);
  wb_distance_sensor_enable(ps(i),TIME_STEP);
end

% Calling MATLAB desktop versionwb_differential_wheels_set_speed(1, -1);
% desktop;

% main loop:
% perform simulation steps of TIME_STEP milliseconds
% and leave the controll to the keyboard
x = 0;
y = 0;
phi = 3.927*pi; 
vleft = 0; 
vright = 0;
while wb_robot_step(TIME_STEP) ~= -1

  % read all distance sensors
       for i=1:N
           sensor_values(i) = wb_distance_sensor_get_value(ps(i));
       end
  % display all distance sensors values
       sensor_values
  left_speed=1;
  right_speed=1;
  %set_speeds(left_speed,right_speed);
  
  left = sensor_values(1)+sensor_values(2)+sensor_values(3);
  right = sensor_values(4)+sensor_values(5)+sensor_values(6);
  
  if left >50 && right >50
      if left > right
          %turn right
          vleft = 2;
          vright = -1;
      else
          vleft = -1;
          vright = -2;
      end 
  elseif left>100
     vleft = 1;
     vright = -1;
  elseif right>100
     vleft = -1;
     vright = 1;
  else
     vleft = 1;
     vright = 1; 
  end;
  
  wb_differential_wheels_set_speed(vleft, vright);
  [x,y,phi] = odometry(vleft, vright,x ,y , phi, -1)
  
end