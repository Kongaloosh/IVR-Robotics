function[x,y,phi] = odometry(vleft, vright, x, y, phi, r)
% if you want to have the distance, you must multiply the results by 2PIR
    if (r == 0)
        r = .053/2;
    end 
%φ �? φ + Δφ = φ - 0.5*(vleft - vright)/(2R)
    %φ �? φ + Δφ = φ
    phi = phi + 0.5 * (vleft - vright) / (2 * r);
    phi = mod(phi, (2*pi));
    
% x �? x + Δx =x + 0.5*(vleft + vright) cos(φ)
    % x �? x + Δx 
    x = x + 0.5 * (vleft+vright) * cos(phi);
    
% y �? y + Δy =y + 0.5*(vleft + vright) sin(φ)
    % y �? y + Δy =y 
    y = y + 0.5 * (vleft + vright) * sin(phi);
end
