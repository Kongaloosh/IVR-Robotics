function wallFollow()

disp('Starting now!');

sensorLeft = wb_distance_sensor_get_value(1);
sensorRight = wb_distance_sensor_get_value(6);
sensorFrontLeft = wb_distance_sensor_get_value(3);
sensorFrontRight = wb_distance_sensor_get_value(4);
  
  
while(sensorFrontLeft < 700 && sensorFrontRight < 700 && sensorLeft < 700)
    disp('Moving forward');
    disp(strcat('Front Left:',{' '}, num2str(sensorFrontLeft)));
  	wb_differential_wheels_set_speed(8,8);
  	wb_robot_step(64);
    sensorLeft = wb_distance_sensor_get_value(1);
    sensorFrontLeft = wb_distance_sensor_get_value(3);
    sensorFrontRight = wb_distance_sensor_get_value(4);
end

disp('Leaving first while loop');
while 1 % while the world is active
 %$ disp(sensor2);   
  % get the values of all the range sensors     
  sensorLeft = wb_distance_sensor_get_value(1);
  sensorRight = wb_distance_sensor_get_value(6);
  sensorFrontLeft = wb_distance_sensor_get_value(3);
  sensorFrontRight = wb_distance_sensor_get_value(4);
  
%   sensor2 = wb_distance_sensor_get_value(2);
%   sensor5 = wb_distance_sensor_get_value(5);
%   sensor7 = wb_distance_sensor_get_value(7);
%   sensor8 = wb_distance_sensor_get_value(8);

  leftTooClose =  800;  
  leftClose = 600;
  leftFar = 400;
  noDetection = 0;
  
  %disp(sensor2);  
  %designed for wall following on the left
  % nothing in front & left side is within desired distance window
  if(sensorFrontLeft == noDetection && sensorFrontRight == noDetection ...
          && sensorLeft < leftClose && sensorLeft > leftFar)
  	wb_differential_wheels_set_speed(3,3);
  	wb_robot_step(64);
    disp('forward');
    following = 1; 
 % nothing in front, left, and right
 %nothing is detected all around, so go forward quickly
 elseif(sensorFrontLeft == noDetection && sensorFrontRight == noDetection ...
         && sensorLeft ==noDetection && sensorRight == noDetection)
    wb_differential_wheels_set_speed(6,6);
  	wb_robot_step(64);
    disp('quick forward');
    following = 0;
  %nothing in front & left wall is sensed but too far away
  %getting further away from wall than desired, so go forward and left
  elseif(sensorFrontLeft == noDetection && sensorFrontRight == noDetection ...
          && sensorLeft < leftFar && sensorLeft > noDetection)
    wb_differential_wheels_set_speed(2,3);
  	wb_robot_step(64);
    disp('left forward');
    following = 1;
  %nothing in front, left wall is close but not too close 
  %starting to get closer than desired, so go forward and right
  elseif(sensorFrontLeft == noDetection && sensorFrontRight == noDetection ...
          && sensorLeft > leftClose && sensorLeft<leftTooClose)
     wb_differential_wheels_set_speed(3,2);
  	wb_robot_step(64);
     disp('right forward');
     following = 1;
   %nothing in front, but left wal is too close
   %back up a bit, then turn slightly
   elseif(sensorFrontLeft == noDetection && sensorFrontRight == noDetection ...
              &&  sensorLeft > leftTooClose)
    wb_differential_wheels_set_speed(-3,-3);
  	wb_robot_step(64);
    pause(1);   
    wb_differential_wheels_set_speed(1,-1);
  	wb_robot_step(64);
    pause(.3);
    disp('spin round');
    following = 1;
  %something is in front of both sensors, so back up and turn slightly
  %currently same actions as previous elseif statement
  elseif(sensorFrontLeft ~= noDetection || sensorFrontRight ~= noDetection)
    wb_differential_wheels_set_speed(-3,-3);
  	wb_robot_step(64);
    pause(1);   
    wb_differential_wheels_set_speed(3,-3);
  	wb_robot_step(64);
    pause(.3);
    disp('spin round');
    following = 1;
  end

end
 
