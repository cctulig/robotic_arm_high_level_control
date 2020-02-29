
function [ theta ] = ikin( x, y, z )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

%task space is 254mm x 304mm x 304mm


l1 = 135;
l2 = 175;
l3 = 169.28;
l4 = sqrt(x^2 + y^2 + (z-l1)^2);
l5 = z-l1;
l6 = sqrt(x^2 + y^2);

theta(1) = atan2(y, x);
disp(theta(1))
theta(2) = (atan2(l5, l6) + acos((l2^2+l4^2-l3^2)/(2*l2*l4)));
disp(theta(2))
theta(3) = -1*((pi/2) - acos((l2^2+l3^2-l4^2)/(2*l2*l3)));
disp(theta(3))

if((theta(1)> 1.54) || (theta(1)< -1.515) || (theta(2)> 1.725) || (theta(2)< -.14) || (theta(3)> 3.85) || (theta(3)< -.55))
    %error('Out of Joint Range');
end

if(z< -45 || z > 260 || x < -60 || x > 300 || y < -230 || y > 230)
    error('Out of Bounds');
end


end