function sMove()

 while 1 % while the world is active
     
  % get the values of all the range sensors     
  sensor_Left = wb_distance_sensor_get_value(1);
  sensor2 = wb_distance_sensor_get_value(2);
  sensor_FrontLeft = wb_distance_sensor_get_value(3);
  sensor_FrontRight = wb_distance_sensor_get_value(4);
  sensor5 = wb_distance_sensor_get_value(5);
  sensor_Right = wb_distance_sensor_get_value(6);
  sensor7 = wb_distance_sensor_get_value(7);
  sensor8 = wb_distance_sensor_get_value(8);

  close_threshold = 800;
  far_threshold = 700;

  % if we're close to a wall
  if (sensor_FrontLeft > far_threshold && sensor_FrontRight > far_threshold)
	% turn so we're facing a wall	
	wb_differential_wheels_set_speed(3,-3);
  % if we're too far from the wall	
  if (sensorLeft > far_threshold)
	% turn so we're facing a wall	
	wb_differential_wheels_set_speed(3,-3);
  % if we're too close to the wall
  if (sensorRight > close_threshold )
  	% turn away from the wall	
	wb_differential_wheels_set_speed(-3,3);
  end

% allow time between timesteps  
pause(1);
     
end
 
