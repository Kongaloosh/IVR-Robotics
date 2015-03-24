function part1()
disp('Starting now!');

% Getting Nodes
gps = wb_robot_get_device('gps');

%Sampling time
TIME_STEP = 32;

% Actuation Flags
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

% Defining the true values, as found by the GPS
xTrue = 0;
yTrue = 0;
zTrue = 0;

phi = pi; 

xLastPosition = 0;
yLastPosition = 0;

vLeft = 1; 
vRight = 1;
sensorTally=0;

EMA = 0;

for i=1:100
    for(k=1:8)
        sensorTally = sensorTally+wb_distance_sensor_get_value(k);
    end
    
    while(sensorTally<tooClose)
        wb_differential_wheels_set_speed(vLeft, vRight);
        [x,y,phi] = odometry(vLeft, vRight,x ,y , phi, -1);
        wb_robot_step(64); 
        sensorTally=0;
        for k=1:8
           sensorTally = sensorTally+wb_distance_sensor_get_value(k);
        end
    end

    xWall = x;
    yWall = y;
    phiWall=phi;


    for i = 1:500
   
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
  
  ret = wb_gps_get_values(gps);
  xTrue = ret(1);
  yTrue = ret(3);
  
  ret = wb_compass_get_values(compass)
  
  
  wb_robot_step(64); %%needed here or the sensors won't read correctly!
  position = sprintf('x: %d, y: %d, phi: %d', x, y,phi); 
  disp(position);
  
  %%check position -> if x & y position are similar, increment errorFlag
  if(xLastPosition < floor(x) + 3 && xLastPosition > floor(x)-3) && ...
          (y < floor(y) + 3 && yLastPosition > floor(y)-3) 
      errorFlag = errorFlag + 1;
  else  %%position isn't similar, so update saved position and reset flag
      xLastPosition = floor(x);
      yLastPosition = floor(y);
      errorFlag=0;
  end

  
  if errorFlag > 50 %%position similar for too long - do something different
      disp('The cake is a lie!');
      errorFlag = 0;
      vLeft = reverseNorm;
      vRight = reverseNorm;
      wb_differential_wheels_set_speed(vLeft, vRight);
      wb_robot_step(64);
      pause(1);
      vLeft = forwardNorm;
      vRight = reverseNorm;
      wb_differential_wheels_set_speed(vLeft, vRight);
      wb_robot_step(64);
      pause(.5);
      
  end
  
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
  elseif(sensorFrontLeft < far && sensorFrontRight < far ...
          && sensorLeftBack > close && sensorLeftBack<closer ...
          && sensorLeftForward <tooClose)
    vLeft = forwardNorm;
    vRight = forwardNorm-1;
    controlInfo = sprintf ('Moving Right & Forward! Left Wheel: %d Right Wheel: %d', vLeft, vRight);
    followFlag = following;
   
   %nothing in front, but left wall is too close
   %turn slightly
   elseif(sensorFrontLeft == noDetection && sensorFrontRight == noDetection ...
              &&  sensorLeftBack > closer)
    vLeft = forwardMed;
    vRight = reverseMed;
    controlInfo = sprintf ('Spinning around! Left Wheel: %d Right Wheel: %d', vLeft, vRight);
    followFlag = following;
  
  %something is in front of both sensors,   %or something close to side/front, so turn sharply
  elseif(sensorFrontLeft > close || sensorFrontRight > close ...
          || sensorLeftForward > close || sensorRightForward > close)
    vLeft = forwardNorm;
    vRight = reverseNorm;
    controlInfo = sprintf ('Spinning around! Left Wheel: %d Right Wheel: %d', vLeft, vRight);
    followFlag = following;
  else %%in a situation not accounted for. normally not a problem - bot just doing what it was doing
      controlInfo = sprintf ('Just keep swimming!!'); 
  end
  
  disp(controlInfo);
  wb_differential_wheels_set_speed(vLeft, vRight);
  [x,y,phi] = odometry(vLeft, vRight,x ,y , phi, -1);

end

test(x,y, phi, xWall, yWall);
end
