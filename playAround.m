function playAround()

wb_differential_wheels_enable_encoders(64);
wb_differential_wheels_set_encoders(0,0);

x=0;
y=0;
phi=pi;

startX = 0;
startY= 0;

vLeft=1;
vRight =1;
oldLeft = 0;
oldRight = 0;

for i=1:40
    wb_differential_wheels_set_speed(vLeft,vRight);
    wb_robot_step(64);
    newLeft = wb_differential_wheels_get_left_encoder;
    newRight = wb_differential_wheels_get_right_encoder;
    [x,y,phi] = odometry(vLeft, vRight,x ,y , phi, 0);
    %[x,y,phi] = encoderOdo(x,y,phi,newLeft-oldLeft, newRight-oldRight);
    oldLeft = newLeft;
    oldRight = newRight;

    position = sprintf('Encoders => x: %d, y: %d, phi: %d', x, y,phi); 
    disp(position);
end
botStop;
newPhi = abs(phi-pi);
while not (phi>newPhi-0.1 && phi<newPhi+0.1)
    vLeft = 3;
    vRight = -3;
    wb_differential_wheels_set_speed(vLeft,vRight);
    wb_robot_step(64);
    [x,y,phi] = odometry(vLeft, vRight,x ,y , phi, 0);
     position = sprintf('Encoders => x: %d, y: %d, phi: %d, newPhi: %d', x, y,phi,newPhi); 
    disp(position);
end
botStop;

vLeft = 1;
vRight = 1;
while (not (floor(x)>=floor(startX)-1 && floor(x)<=floor(startX)+1)) ||...
        not (floor(y)>=floor(startY)-1 && floor(y)<=floor(startY)+1)
    wb_differential_wheels_set_speed(vLeft,vRight);
    wb_robot_step(64);
    newLeft = wb_differential_wheels_get_left_encoder;
    newRight = wb_differential_wheels_get_right_encoder;
    [x,y,phi] = odometry(vLeft, vRight,x ,y , phi, 0);
    %[x,y,phi] = encoderOdo(x,y,phi,newLeft-oldLeft, newRight-oldRight);
    oldLeft = newLeft;
    oldRight = newRight;

    position = sprintf('Encoders => x: %d, y: %d, phi: %d', x, y,phi); 
    disp(position);
    position = sprintf('FloorX : %d, floorY: %d', floor(x), floor(y));
    disp(position);
end


botStop;
end