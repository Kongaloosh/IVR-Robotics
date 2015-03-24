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
phi = 1.25*pi; 
vleft = 0; 
vright = 0;
detected = 0;

alpha = 0.9;
EMA = 0;
pause = true;  % 0 for left 1 for right



while wb_robot_step(detected == 0) 
    all = 0;
  % read all distance sensors
       for i=1:N
           sensor_values(i) = wb_distance_sensor_get_value(ps(i));
           all = all + wb_distance_sensor_get_value(ps(i));
       end
  % display all distance sensors values
  
  sensor_values
  
  left_speed=1;
  right_speed=1;
  
  left = sensor_values(1)+sensor_values(2);
  right = sensor_values(4)+sensor_values(5);
  front_left = sensor_values(3);
  front_right = sensor_values(4);
  
    % we take the exponential moving average of all the values
  EMA = (alpha * all) + (1.0 - alpha) * EMA
  
  % if we're blocked on both sides, trigger evasive maneuver
  if left >50 && right >50
      blocked = 1;
      while(blocked == 1) 
          disp(blocked)
          for i=1:N
            sensor_values(i) = wb_distance_sensor_get_value(ps(i));
            all = all + wb_distance_sensor_get_value(ps(i));
          end    
          
        if (front_left + front_right > left)
            left_speed=1;
            right_speed=-1;
        elseif (front_left + front_right > right)
            left_speed=-1;
            right_speed=1;
        else
            left_speed=1;
            right_speed=1;
        end
        
        wb_differential_wheels_set_speed(vleft, vright);
        [x,y,phi] = odometry(vleft, vright,x ,y , phi, -1);
        wb_robot_step(64);
        
       if left <50 || right <50
            blocked == 0 
       end
      end
      
      % If we're not close to the walls
      if ((y > -850 || x < -200))
          disp('intruder')
      end
      
  elseif left+front_left > 10
     vleft = 1;
     vright = -1;
  elseif right + front_right > 10
     vleft = -1;
     vright = 1;
  else
     vleft = 1;
     vright = 1; 
  end;
  
  wb_differential_wheels_set_speed(vleft, vright);
  [x,y,phi] = odometry(vleft, vright,x ,y , phi, -1);
  
end