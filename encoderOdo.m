function [xNew,yNew,newPhi] = encoderOdo(x,y, phi, deltaLeft, deltaRight)

positionChange = (deltaLeft+deltaRight)/2;
deltaPhi = (deltaRight-deltaLeft)/ (2*4);
newPhi = deltaPhi+phi;

xDelta = positionChange * cos(phi+deltaPhi/2);
yDelta = positionChange * sin(phi+deltaPhi/2);

xNew = x + xDelta;
yNew = y + yDelta;

end
