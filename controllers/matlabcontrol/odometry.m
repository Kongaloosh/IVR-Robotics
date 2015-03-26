function[x,y,phi] = odometry(vleft, vright, x, y, phi, r)
% odometry based on velocity commands

% if you want to have the distance, you must multiply the results by 2PIR
    r_wheel = 0.008;
    d_axel = 0.053;

    vright = vright * 2 * pi * r_wheel;
    vleft = vleft * 2 * pi * r_wheel;

% x ← x + Δx =x + 0.5*(vleft + vright) cos(φ)
    % x ← x + Δx 
    x = x + 0.5 * (vleft + vright) * cos(phi);
    
% y ← y + Δy =y + 0.5*(vleft + vright) sin(φ)
    % y ← y + Δy =y 
    y = y + 0.5 * (vleft + vright) * sin(phi);

%φ ← φ + Δφ = φ - 0.5*(vleft - vright)/(2R)
    %φ ← φ + Δφ = φ
    phi = phi + 0.5 * (vleft - vright) / (d_axel);
end
