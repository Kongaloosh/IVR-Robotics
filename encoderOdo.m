function [xNew,yNew,newPhi] = encoderOdo(x,y, phi, deltaLeft, deltaRight)

%WHEEL_CIRC = 2*pi*.008;
ticksPerRev = 100*pi*2;
r=.008;
distanceLeft=(deltaLeft/100)*r;
distanceRight=(deltaRight/100)*r;


positionChange = (distanceLeft+distanceRight)/2;
xNew = x - positionChange * sin(phi);
yNew = y - positionChange * cos(phi);

newPhi = phi+ (xNew - yNew)/.053;

end
