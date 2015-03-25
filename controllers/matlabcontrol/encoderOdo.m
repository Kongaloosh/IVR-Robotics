function [xNew,yNew,newPhi] = encoderOdo(x,y, phi, deltaLeft, deltaRight)




positionChange = (deltaLeft+deltaRight)/2;
disp(positionChange);
deltaPhi = (deltaRight-deltaLeft)/ (2*4);
newPhi = deltaPhi+phi;
disp(newPhi);

xDelta = positionChange * cos(phi+deltaPhi/2);
yDelta = positionChange * sin(phi+deltaPhi/2);

xNew = x + xDelta;
yNew = y + yDelta;

end
