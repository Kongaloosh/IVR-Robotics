function[newx,newy,newphi] = odometry2(lastx, lasty, deltaencoderx, deltaencodery, phi)
% odometry based on encoder values
    
% constants based on the simulation
    r_wheel = 0.008;
    d_axel = 0.053;
    encoder_resolution = 100;
    
    % calculate distance travelled, as percieved by encoders
    % divide by encoder_res to get the angle
    % multiply by the radius to get the arc, or distance traveled
    distancex = (deltaencoderx / encoder_resolution) * r_wheel;
    distancey = (deltaencodery / encoder_resolution) * r_wheel;

    % calculate avg distance travelled
    avgdistance = distancex+distancey/2;

    % calcuate new x and y positions
    newx = lastx - avgdistance * sin(phi);
    newy = lasty - avgdistance * cos(phi);

    % calculate phi
    newphi = phi + (distancex - distancey)/d_axel;

end
