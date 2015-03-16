function[x,y,phi] = odometry(vleft, vright, x, y, phi, r)
% x ← x + Δx =x + 0.5*(vleft + vright) cos(φ)
    % x ← x + Δx 
    x = x + 0.5 * (vleft+vright) * cos(phi)
    
% y ← y + Δy =y + 0.5*(vleft + vright) sin(φ)
    % y ← y + Δy =y 
    y = y + 0.5 * (vleft + vright) * sin(phi)
    
%φ ← φ + Δφ = φ - 0.5*(vleft - vright)/(2R)
    %φ ← φ + Δφ = φ
    phi = phi + 0.5 * (vleft - vright) / (2 * r)