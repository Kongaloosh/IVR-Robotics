function wallFollow()

disp('Starting now!');

followFlag = 0; 
errorFlag = 0;

%define thresholds for bot use
tooClose =  800;  
leftClose = 600;
leftFar = 400;
noDetection = 0;
close = 500;

%define default speed
normSpd = 3;
reverseNormSpd = -3;


wb_differential_wheels_set_speed(normSpd, normSpd);

while 1 %bot has approached something. while world is active, keep looping
 
  % get the values of all the range sensors    
  % get speed values from both wheels
  sensorLeftBack = wb_distance_sensor_get_value(1);
  sensorLeftForward = wb_distance_sensor_get_value(2);
  sensorFrontLeft = wb_distance_sensor_get_value(3);
  sensorFrontRight = wb_distance_sensor_get_value(4);
  sensorRightForward = wb_distance_sensor_get_value(5);
  sensorRightBack = wb_distance_sensor_get_value(6);
  sensorBackRight = wb_distance_sensor_get_value(7);
  sensorBackLeft = wb_distance_sensor_get_value(8);

  wb_robot_step(64); %%needed here or the sensors won't read correctly!
  
  
  disp(sensorLeftBack);
   disp(sensorLeftForward);
   disp(sensorFrontLeft);
    disp(sensorFrontRight);
     disp(sensorRightForward);
      disp(sensorRightBack);
       disp(sensorBackRight);
        disp(sensorBackLeft);
  
  
  if errorFlag == 5
     wb_differential_wheels_set_speed(reverseNormSpd,reverseNormSpd);
  	wb_robot_step(64);
    pause(.4);
    wb_differential_wheels_set_speed(reverseNormSpd,normSpd);
  	wb_robot_step(64);
    pause(.4);
    disp('must be stuck!');
    errorFlag = 0;
  end
  
  
  %designed for wall following on the left
  % nothing in front & left side is within desired distance window
  if(sensorFrontLeft == noDetection && sensorFrontRight == noDetection ...
          && sensorLeftBack < leftClose && sensorLeftBack > leftFar ...
          && sensorLeftForward < tooClose && sensorRightForward < tooClose)
    wb_differential_wheels_set_speed(normSpd,normSpd);
  	wb_robot_step(64);
    disp('forward');
    followFlag = 1;
      
  %nothing in front, left, and right
  %nothing is detected all around, so go forward quickly
  elseif(sensorFrontLeft == noDetection && sensorFrontRight == noDetection ...
         && sensorLeftBack == noDetection && sensorRightBack == noDetection)
    if(followFlag == 1)
        disp('Huh? Was just following something, now its gone!')  
        wb_differential_wheels_set_speed(reverseNormSpd,normSpd);
        wb_robot_step(64);
        pause(.5)
        followFlag = 0;
    else
        disp('Moving forward');
        wb_differential_wheels_set_speed(normSpd, normSpd);
        wb_robot_step(64);
        
        followFlag = 0;
     end

  %nothing in front & left wall is sensed but too far away
  %getting further away from wall than desired, so go forward and left
  elseif(sensorFrontLeft == noDetection && sensorFrontRight == noDetection ...
          && sensorLeftBack < leftFar && sensorLeftBack > noDetection)
    wb_differential_wheels_set_speed(normSpd - ((leftFar-sensorLeftBack)/100), normSpd);
  	wb_robot_step(64);
    disp('left forward');
    followFlag = 1;
    
  %nothing in front, left wall is close but not too close 
  %starting to get closer than desired, so go forward and right
  elseif(sensorFrontLeft == noDetection && sensorFrontRight == noDetection ...
          && sensorLeftBack > leftClose && sensorLeftBack<tooClose ...
          && sensorLeftForward <tooClose)
     wb_differential_wheels_set_speed(normSpd,normSpd - 1);
  	wb_robot_step(64);
     disp('right forward');
     followFlag = 1;
   
   %nothing in front, but left wall is too close
   %back up a bit, then turn slightly
   elseif(sensorFrontLeft == noDetection && sensorFrontRight == noDetection ...
              &&  sensorLeftBack > tooClose)
    wb_differential_wheels_set_speed(2,-2);
  	wb_robot_step(64);
    pause(.4);
    disp('spin round');
    followFlag = 1;
  
  %something is in front of both sensors, 
  %or something close to side/front, so turn sharply
  elseif(sensorFrontLeft > tooClose || sensorFrontRight > tooClose ...
          || sensorLeftForward > tooClose || sensorRightForward > tooClose)
    wb_differential_wheels_set_speed(normSpd, reverseNormSpd);
  	wb_robot_step(64);
    pause(.2);
    disp('spin round');
    followFlag = 1;
  else disp ('DANGER WILL ROBINSON!');  %%in a situation not accounted for
        disp(wb_distance_sensor_get_value(6))
  end
  

end
 
