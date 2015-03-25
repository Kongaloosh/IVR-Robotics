function test1()
wb_differential_wheels_enable_encoders(64);
wb_differential_wheels_set_encoders(0,0);
disp('Starting now!');


vLeft = 1;
vRight = 1;
x = 0;
y=0;
phi=0;

xOdo = 0;
yOdo = 0;
phiOdo = 0;

for k=1:100
wb_differential_wheels_set_speed(vLeft, vRight);
wb_robot_step(64);
deltaLeft = wb_differential_wheels_get_left_encoder/680;
deltaRight = wb_differential_wheels_get_right_encoder/680;
[x,y,phi] = encoderOdo(x,y,phi,deltaLeft, deltaRight);
[xOdo,yOdo,phiOdo] = odometry(vLeft, vRight,xOdo ,yOdo , phiOdo, 0);
end

botStop;
update = sprintf('Encoders=> x: %d, y: %d, phi: %d', x, y, phi);
updateOdo = sprintf('Odometry=> x: %d, y: %d, phi: %d', xOdo, yOdo, phiOdo);
disp(update);
disp(updateOdo);


end
