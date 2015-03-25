function [xNew,yNew,newPhi] = encoderOdo(x,y, phi, deltaLeft, deltaRight)

WHEEL_CIRC = pi*.0008;
ticksPerRev = 100*pi*2;

% deltaLeft = (deltaLeft/6480)*WHEEL_CIRC;
% deltaRight = (deltaRight/6480)*WHEEL_CIRC;
distanceLeft = (deltaLeft/ticksPerRev)*pi*.053;
distanceRight = (deltaRight/ticksPerRev)*pi*.053;
positionChange = (deltaLeft+deltaRight)/2;
deltaPhi = (deltaRight-deltaLeft)/ (.053);
newPhi = deltaPhi+phi;

xDelta = positionChange * cos(phi+deltaPhi/2);
yDelta = positionChange * sin(phi+deltaPhi/2);

xNew = x + xDelta;
yNew = y + yDelta;

% xNew = x + positionChange*cos(phi);
% yNew = y + positionChange*sin(phi);


end
