function[newx,newy,newphi] = odometry2(lastx, lasty, deltaencoderx, deltaencodery, phi)
% if you want to have the distance, you must multiply the results by 2PIR

    r_wheel = 0.008;
    r_axel = 0.053/2;
    % calculate distance travelled, as percieved by encoders
    distancex = (deltaencoderx / 100) * r_wheel;
    distancey = (deltaencodery / 100) * r_wheel;

    % calculate avg distance travelled
    avgdistance = distancex+distancey/2;

    % calcuate new x position
    newx = lastx - avgdistance * sin(phi);
    newy = lasty - avgdistance * cos(phi);

    % calculate phi

    newphi = phi + (newx - newy)/2*r_axel;

end
