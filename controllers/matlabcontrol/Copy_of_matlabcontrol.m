% ======================================================================
%                               SETUP 
% ======================================================================

TIME_STEP = 64; % rate at which we get new drops from the sensors
N = 8;

% get and enable distance sensors 
for i=1:N
  ps(i) = wb_robot_get_device(['ds' int2str(i-1)]);
  wb_distance_sensor_enable(ps(i),TIME_STEP);
end

% get and enable the camera
%camera = wb_robot_get_device('camera');
%wb_camera_enable(camera, TIME_STEP);

% get and enable encoders

%wb_differential_wheels_enable_encoders(64);
%wb_differential_wheels_set_encoders(0,0);

% get and enable the GPS
%gps = wb_robot_get_device('camera');
%wb_gps_enable(gps, TIME_STEP)

% ======================================================================
%                           CONTROL LOOP
% ======================================================================


while wb_robot_step(TIME_STEP) ~= -1

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
x = 0;
y = 0;
phi = pi; 
xLastPosition = 0;
yLastPosition = 0;

%define x,y,&phi for encoder odometry readings
xEnc = 1;
yEnc = 1;
phiEnc = pi;

%set initial velocity of left and right wheels to 1
vLeft = 1; 
vRight = 1;
sensorTally=0; %%used for a sum of all distance sensor readings

pidControl = pid(1/100); %%just use P for now
distanceError = 0; 
Kp = 1/100; %constant for porpotionality
Kd = 0; %%for derivative control implementation
distanceDelta = 0; %change in left back sensor readings
pdControlFunction =0;

%just starting off, head straight on until the bot gets near an object
for(k=1:8)
   sensorTally = sensorTally+wb_distance_sensor_get_value(k);
end
while(sensorTally<tooClose)
    wb_differential_wheels_set_speed(vLeft, vRight);
    [x,y,phi] = odometry(vLeft, vRight,x ,y , phi, 2.5);
    %deltaLeft = wb_differential_wheels_get_left_encoder/680;
    %deltaRight = wb_differential_wheels_get_right_encoder/680;
    [xEnc,yEnc,phiEnc] = encoderOdo(xEnc,yEnc,phiEnc,deltaLeft, deltaRight);
    wb_robot_step(64); 
    sensorTally=0;
    for k=1:8
       sensorTally = sensorTally+wb_distance_sensor_get_value(k);
    end
end

%just found a wall or other object, mark this spot for later return 
xWall = x;  
yWall = y;
phiWall=phi;
xWallEnc = xEnc;
yWallEnc = yEnc;

lastSensorLeftBack = wb_distance_sensor_get_value(1); %%start reading for sensorLeftBack

for i = 1:500  %%arbitrary wallFollowing end point
   
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
  position = sprintf('Encoders => x: %d, y: %d, phi: %d', xEnc, yEnc,phiEnc); 
  disp(position);
  
  %%check position -> if x & y position are similar, increment errorFlag
  %need to convert this to use accelerometers
  if(xLastPosition < floor(x) + 3 && xLastPosition > floor(x)-3) && ...
          (y < floor(y) + 3 && yLastPosition > floor(y)-3) 
      errorFlag = errorFlag + 1;
  else  %%position isn't similar, so update saved position and reset flag
      xLastPosition = floor(x);
      yLastPosition = floor(y);
      errorFlag=0;
  end
 
  %designed for wall following on the left
  %nothing in front & left side is within desired distance window
  if(sensorFrontLeft < far && sensorFrontRight < far ...
            && sensorLeftForward < tooClose && sensorRightForward < tooClose ...
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
          || sensorLeftForward > tooClose || sensorRightForward > tooClose)
    vLeft = forwardNorm;
    vRight = reverseNorm;
    controlInfo = sprintf ('Spinning around! Left Wheel: %d Right Wheel: %d', vLeft, vRight);
    followFlag = following;
  else %%in a situation not accounted for. normally not a problem - bot just doing what it was doing
      controlInfo = sprintf ('Just keep swimming!!'); 
  end
  
  % display control info
  disp(controlInfo);
  
  % send commands
  wb_differential_wheels_set_speed(vLeft, vRight);
  
  % calculate values from the commands
  [x,y,phi] = odometry(vLeft, vRight,x ,y , phi, 0);
  
  % caluculate values from the encoders
  %deltaLeft = wb_differential_wheels_get_left_encoder/680;
  %deltaRight = wb_differential_wheels_get_right_encoder/680; 
  [xEnc,yEnc,phiEnc] = encoderOdo(xEnc,yEnc,phiEnc,deltaLeft, deltaRight);

end

%test(x,y, phi, xWall, yWall);
testEnc(xEnc, yEnc, phiEnc, xWallEnc, yWallEnc);
end