function[x,y,phi] = odometry(vleft, vright, x, y, phi, r)
% if you want to have the distance, you must multiply the results by 2PIR
    if (r == 0)
        r = 0.053;
    end 
%     vleft = vleft *.10472*9.54*.004;
%     vright = vright *.10472*9.54*.004
vright = vright * 2 * pi * 0.02;
vleft = vleft * 2 * pi * 0.02;

% x ← x + Δx =x + 0.5*(vleft + vright) cos(φ)
    % x ← x + Δx 
    x = x + 0.5 * (vleft+vright) * cos(phi);
    
% y ← y + Δy =y + 0.5*(vleft + vright) sin(φ)
    % y ← y + Δy =y 
    y = y + 0.5 * (vleft + vright) * sin(phi);

%φ ← φ + Δφ = φ - 0.5*(vleft - vright)/(2R)
    %φ ← φ + Δφ = φ
    phi = phi + (vleft - vright) / (r);
end
