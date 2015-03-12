function sMove()

 while 1 % while the world is active
     
  % get the values of all the range sensors     
  sensorLeft = wb_distance_sensor_get_value(1);
  sensor2 = wb_distance_sensor_get_value(2);
  sensorFrontLeft = wb_distance_sensor_get_value(3);
  sensorFrontRight = wb_distance_sensor_get_value(4);
  sensor5 = wb_distance_sensor_get_value(5);
  sensor_Right = wb_distance_sensor_get_value(6);
  sensor7 = wb_distance_sensor_get_value(7);
  sensor8 = wb_distance_sensor_get_value(8);

  closeThreshold = 800;
  farThreshold = 700;
  % if there's nothing around you
  if(sensorFrontLeft < farThreshold && sensorFrontRight < farThreshold && sensorLeft < farThreshold)
  	wb_differential_wheels_set_speed(3,-3);
  	wb_robot_step(64);
        pause(1);
  % if we're close to a wall
  if (sensorFrontLeft > farThreshold && sensorFrontRight > farThreshold)
	% turn so we're facing a wall	
	wb_differential_wheels_set_speed(3,-3);
  	wb_robot_step(64);
        pause(1);
  % if we're too far from the wall	
  else if (sensorLeft > farThreshold)
	% turn so we're facing a wall	
	wb_differential_wheels_set_speed(3,-3);
  	wb_robot_step(64);
        pause(1);
  % if we're too close to the wall
  else if (sensorRight > closeThreshold )
  	% turn away from the wall	
	wb_differential_wheels_set_speed(-3,3);
  	wb_robot_step(64);
        pause(1);
  end

% allow time between timesteps  
pause(1);
     
end
 
