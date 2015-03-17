function wallFollow()

disp('Starting now!');

%%Get an initial reading
sensorStartLeftBack = wb_distance_sensor_get_value(1);
sensorStartRightBack = wb_distance_sensor_get_value(6);
sensorStartFrontLeft = wb_distance_sensor_get_value(3);
sensorStartFrontRight = wb_distance_sensor_get_value(4);
  
  

followFlag = 0; 

%define thresholds for bot use
tooClose =  800;  
leftClose = 600;
leftFar = 400;
noDetection = 0;
close = 500;

%define default speed
normSpd = 3;
reverseNormSpd = -3;
  
%Just starting off, if not sensing anything, floor it
while(sensorStartLeftBack + sensorStartRightBack + sensorStartFrontLeft + sensorStartFrontRight < 50)
    disp('Moving forward');
   	wb_differential_wheels_set_speed(normSpd+5,normSpd+5);
  	wb_robot_step(64);
    sensorStartLeftBack = wb_distance_sensor_get_value(1);
    sensorStartFrontLeft = wb_distance_sensor_get_value(3);
    sensorStartFrontRight = wb_distance_sensor_get_value(4);
    sensorStartRightBack = wb_distance_sensor_get_value(6);
    
end

disp('Leaving first while loop');

while 1 %bot has approached something. while world is active, keep looping
 
  % get the values of all the range sensors     
  sensorLeftBack = wb_distance_sensor_get_value(1);
  sensorLeftForward = wb_distance_sensor_get_value(2);
  sensorFrontLeft = wb_distance_sensor_get_value(3);
  sensorFrontRight = wb_distance_sensor_get_value(4);
  sensorRightForward = wb_distance_sensor_get_value(5);
  sensorRightBack = wb_distance_sensor_get_value(6);
  sensorBackRight = wb_distance_sensor_get_value(7);
  sensorBackLeft = wb_distance_sensor_get_value(8);
  wb_robot_step(64); %%needed here or the sensors won't read correctly!
  

  
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
        wb_differential_wheels_set_speed(-3,3);
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
  
  %something is in front of both sensors, so back up and turn 
  %currently same actions as previous elseif statement
  elseif(sensorFrontLeft > tooClose || sensorFrontRight > tooClose ...
          || sensorLeftForward > close || sensorRightForward > close)
    wb_differential_wheels_set_speed(normSpd, reverseNormSpd);
  	wb_robot_step(64);
    pause(.3);
    disp('back then spin round');
    followFlag = 1;
  else disp ('DANGER WILL ROBINSON!');  %%in a situation not accounted for
        disp(wb_distance_sensor_get_value(4))
  end
  
  



end
 
