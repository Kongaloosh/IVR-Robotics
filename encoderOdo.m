function [xNew,yNew,newPhi] = encoderOdo(x,y, phi, deltaLeft, deltaRight)

WHEEL_CIRC = 2*pi*.0008;

deltaLeft = (deltaLeft/648)*WHEEL_CIRC;
deltaRight = (deltaRight/648)*WHEEL_CIRC;
positionChange = (deltaLeft+deltaRight)/2;
deltaPhi = (deltaRight-deltaLeft)/ (2*4);
newPhi = deltaPhi+phi;

xDelta = positionChange * cos(phi+deltaPhi/2);
yDelta = positionChange * sin(phi+deltaPhi/2);

xNew = x + xDelta;
yNew = y + yDelta;

end
