function [xNew,yNew,newPhi] = encoderOdo(x,y, deltaLeft, deltaRight, phi)
%WHEEL_CIRC = 2*pi*.008;
%ticksPerRev = 100*pi*2;

wheel_r=.008;

distanceLeft=(deltaLeft/100)*wheel_r;
distanceRight=(deltaRight/100)*wheel_r;

positionChange = (distanceLeft+distanceRight)/2;

xNew = x - positionChange * sin(phi);
yNew = y - positionChange * cos(phi);

newPhi = phi+ (xNew -yNew)/.053;
