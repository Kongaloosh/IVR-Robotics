function sMove()

 while 1
     
       
  sensor1 = wb_distance_sensor_get_value(1);
  sensor2 = wb_distance_sensor_get_value(2);
  sensor3 = wb_distance_sensor_get_value(3);
  sensor4 = wb_distance_sensor_get_value(4);
  sensor5 = wb_distance_sensor_get_value(5);
  sensor6 = wb_distance_sensor_get_value(6);
  sensor7 = wb_distance_sensor_get_value(7);
  sensor8 = wb_distance_sensor_get_value(8);

%   if(sensor3 == 0 && sensor4 == 0)
%       wb_differential_wheels_set_speed(6, 6);
%       wb_robot_step(64);
%   elseif (sensor1) < 400
%       wb_differential_wheels_set_speed(3,-3);
%       wb_robot_step(64);
%   elseif (sensor1) > 400
%       wb_differential_wheels_set_speed(-3,3);
%       wb_robot_step(64);
%   end
  wb_differential_wheels_set_speed(3, 3);
      
  if(sensor3 == 0 && sensor4 == 0 && sensor1 == 0)
      wb_differential_wheels_set_speed(6,6);
      wb_robot_step(64);
      pause(1);
  elseif(sensor3 == 0 && sensor4 == 0 && sensor1 > 500)
      wb_differential_wheels_set_speed(3, 3);
      wb_robot_step(64);
      pause(1);
      disp('scooby');
  elseif (sensor3 == 0 && sensor4 == 0 && sensor1 < 500)
      wb_differential_wheels_set_speed(6,4);
      wb_robot_step(64);
      pause(1);
  elseif (sensor3 == 0 && sensor4 == 0)
      wb_differential_wheels_set_speed(6,6);
      wb_robot_step(64);
      pause(1);
  else 
      wb_differential_wheels_set_speed(-6,6);
      wb_robot_step(64); 
      pause(1);
        
  end
  
  
pause(1);
     
end
 