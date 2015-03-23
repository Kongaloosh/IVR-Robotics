function part1()

disp('Starting now!');

following = 1;
notFollowing = 0;
followFlag = following; 
errorFlag = 0;

%define thresholds for bot use
tooClose =  800;  
closer = 600;
far = 400;
noDetection = 0;
close = 500;

%define speeds
forwardNorm = 3;
reverseNorm = -3;
forwardMed = 2;
reverseMed = -2;

%define x,y,&phi for odometry readings
x = 0;
y = 0;
phi = 3.927*pi; 
xLastPosition = 0;
yLastPosition = 0;

vLeft = 0; 
vRight = 0;
wb_differential_wheels_set_speed(vLeft,vRight);

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
  position = sprintf('x: %d, y: %d, phi: %d', x, y,phi); 
  disp(position);
   
  %designed for wall following on the left
  % nothing in front & left side is within desired distance window
  if(sensorFrontLeft == noDetection && sensorFrontRight == noDetection ...
          && sensorLeftBack < close && sensorLeftBack > far ...
          && sensorLeftForward < tooClose && sensorRightForward < tooClose)
    vLeft = forwardNorm;
    vRight = forwardNorm;
    controlInfo = sprintf('Moving forward! Left Wheel: %d Right Wheel: %d', vLeft, vRight); 
    followFlag = following;
      
  %nothing in front, left, and right
  %nothing is detected all around, so go forward quickly
  elseif(sensorFrontLeft == noDetection && sensorFrontRight == noDetection ...
         && sensorLeftBack == noDetection && sensorRightBack == noDetection)
    if(followFlag == 1)
        vLeft = reverseNorm;
        vRight = forwardNorm;
        controlInfo = sprintf ...
            ('Was just following something, now its gone! Turning! Left Wheel: %d Right Wheel: %d',...
            vLeft, vRight);
        followFlag = notFollowing;
    else
        vLeft = reverseNorm;
        vRight = forwardNorm;
        controlInfo = sprintf ('Moving Forward! Left Wheel: %d Right Wheel: %d', vLeft, vRight);
        followFlag = notFollowing;
     end

  %nothing in front & left wall is sensed but too far away
  %getting further away from wall than desired, so go forward and left
  elseif(sensorFrontLeft == noDetection && sensorFrontRight == noDetection ...
          && sensorLeftBack < far && sensorLeftBack > noDetection ...
          && sensorLeftForward < tooClose)
    vLeft = forwardNorm-((far-sensorLeftBack)/100);
    vRight = forwardNorm;
    controlInfo = sprintf ('Moving Left & Forward! Left Wheel: %d Right Wheel: %d', vLeft, vRight);
    followFlag = following;
    
  %nothing in front, left wall is close but not too close 
  %starting to get closer than desired, so go forward and right
  elseif(sensorFrontLeft == noDetection && sensorFrontRight == noDetection ...
          && sensorLeftBack > close && sensorLeftBack<closer ...
          && sensorLeftForward <tooClose)
    vLeft = forwardNorm;
    vRight = forwardNorm-1;
    controlInfo = sprintf ('Moving Right & Forward! Left Wheel: %d Right Wheel: %d', vLeft, vRight);
    followFlag = following;
   
   %nothing in front, but left wall is too close
   %turn slightly
   elseif(sensorFrontLeft == noDetection && sensorFrontRight == noDetection ...
              &&  sensorLeftBack > tooClose)
    vLeft = forwardMed;
    vRight = reverseMed;
    controlInfo = sprintf ('Spinning around! Left Wheel: %d Right Wheel: %d', vLeft, vRight);
    followFlag = following;
  
  %something is in front of both sensors,   %or something close to side/front, so turn sharply
  elseif(sensorFrontLeft > tooClose || sensorFrontRight > tooClose ...
          || sensorLeftForward > close || sensorRightForward > close)
    vLeft = forwardNorm;
    vRight = reverseNorm;
    controlInfo = sprintf ('Spinning around! Left Wheel: %d Right Wheel: %d', vLeft, vRight);
    followFlag = following;
  else %%in a situation not accounted for. normally not a problem - bot just keeps moving forward
      controlInfo = sprintf ('Just keep swimming!!'); 
  end
  
  disp(controlInfo);
  wb_differential_wheels_set_speed(vLeft, vRight);
  [x,y,phi] = odometry(vLeft, vRight,x ,y , phi, -1);

end
 
