function [ p ] = fwkin3001( theta1, theta2, theta3 )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
link1 = [cos(theta1), 0, sin(theta1), 0;
            sin(theta1), 0, -cos(theta1), 0;
            0, 1, 0, 135;
            0, 0, 0, 1;];
link2 = [cos(theta2), -sin(theta2), 0, 175*cos(theta2);
        sin(theta2) cps(theta2), 0, 175*sin(theta2);
        0, 0, 1, 0;
        0, 0, 0, 1;];
link3 = [cos(theta3-pi/2), -sin(theta3-pi/2), 0, 169.28*cos(theta3-pi/2);
        sin(theta3-pi/2), cos(theta3-pi/2), 0, 169.28*sin(theta3-pi/2);
        0, 0, 1, 0;
        0, 0, 0, 1;];
    
    final = (link1 * link2 * link3);
    p = final(1:3, 4);

end

