function test(xStart, yStart, phiStart, xWall, yWall)

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
desiredDistance = 500;


%define speeds
forwardNorm = 3;
reverseNorm = -3;
forwardMed = 2;
reverseMed = -2;

%define x,y,&phi for odometry readings
x = xStart;
y = yStart;
phi = phiStart; 

%set initial velocity of left and right wheels to 1
vLeft = 1; 
vRight = 1;
sensorTally=0; %%used for a sum of all distance sensor readings

pidControl = pid(1/100); %%just use P for now
distanceError = 0; 
Kp = 1/200; %constant for porpotionality
Kd = 1/300; %%for derivative control implementation
distanceDelta = 0; %change in left back sensor readings
pdControlFunction =0;

lastSensorLeftBack = wb_distance_sensor_get_value(1); %%start reading for sensorLeftBack

while (not (floor(x)>floor(xWall)-10 && floor(x)<floor(xWall)+10)) ||...
        not (floor(y)>floor(yWall)-10 && floor(y)<floor(yWall)+10)  %%arbitrary wallFollowing end point
   
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
  distanceError = desiredDistance - sensorLeftBack;
  distanceDelta = lastSensorLeftBack - sensorLeftBack;
  pdControlFunction = distanceError*Kp + distanceDelta*Kd; 
  position = sprintf('Odometry => x: %d, y: %d, phi: %d', x, y,phi); 
  disp(position);
  stopPos = sprintf('Stopping at => x: %d, y: %d', xWall, yWall); 
  disp(stopPos);

  
  %designed for wall following on the left
  %nothing in front & left side is within desired distance window
  if(sensorFrontLeft < far && sensorFrontRight < far ...
            && sensorLeftForward < close && sensorRightForward < close ...
            && sensorLeftBack > noDetection)
    vLeft = forwardNorm;
    vRight = forwardNorm + pdControlFunction;
    controlInfo = sprintf('Moving forward! Left Wheel: %d Right Wheel: %d', vLeft, vRight); 
    followFlag = following;
    
  %nothing in front, left, and right
  %nothing is detected all around, so go forward quickly
  elseif(sensorFrontLeft == noDetection && sensorFrontRight == noDetection ...
         && sensorLeftBack == noDetection && sensorRightBack == noDetection)
    if(followFlag == following)
        vLeft = reverseNorm;
        vRight = forwardNorm;
        controlInfo = sprintf ...
            ('Was just following something, now its gone! Turning! Left Wheel: %d Right Wheel: %d',...
            vLeft, vRight);
        followFlag = following;
    else
        vLeft = forwardNorm;
        vRight = forwardNorm;
        controlInfo = sprintf ('Moving Forward! Left Wheel: %d Right Wheel: %d', vLeft, vRight);
        followFlag = notFollowing;
     end
  
  %something is in front of both sensors,   %or something close to side/front, so turn sharply
  elseif(sensorFrontLeft >= far || sensorFrontRight >=far ...
          || sensorLeftForward > close || sensorRightForward > tooClose ...
          || sensorLeftBack >tooClose)
    vLeft = forwardNorm;
    vRight = reverseNorm;
    controlInfo = sprintf ('Spinning around! Left Wheel: %d Right Wheel: %d', vLeft, vRight);
    followFlag = following;
  else %%in a situation not accounted for. normally not a problem - bot just doing what it was doing
      controlInfo = sprintf ('Just keep swimming!!'); 
  end
  
  disp(controlInfo);
  wb_differential_wheels_set_speed(vLeft, vRight);
  [x,y,phi] = odometry(vLeft, vRight,x ,y , phi, 0);
   

end

 disp('Found Somethin!')
 position = sprintf('Odo => x: %d, y: %d, phi: %d', x, y,phi); 
 disp(position);
 stopPos = sprintf('Stopping at => x: %d, y: %d', xWall, yWall); 
 disp(stopPos);
 botStop;
 
end
