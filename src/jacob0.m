function [ J ] = jacob0( q )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

 l1 = 135;
 l2 = 175;
 l3 = 169.28;

T01 = [cos(q(1)), 0, sin(q(1)), 0;
    sin(q(1)), 0, -cos(q(1)), 0;
    0, 1, 0, l1;
    0, 0, 0, 1;];
T12 = [cos(q(2)), -sin(q(2)), 0, l2*cos(q(2));
    sin(q(2)) cos(q(2)), 0, l2*sin(q(2));
    0, 0, 1, 0;
    0, 0, 0, 1;];
T23 = [sin(q(3)), cos(q(3)), 0, l3*sin(q(3));
    -cos(q(3)), sin(q(3)), 0, -1*l3*cos(q(3));
    0, 0, 1, 0;
    0, 0, 0, 1;];
T02= T01*T12;
% disp('T02')
% disp(T02)
T03=T02*T23;
% disp('T03')
% disp(T03)
Jp1= cross([0 0 1],T03(1:3,4));
Jp2= cross(T01(1:3,3)',(T03(1:3,4)- T01(1:3,4))');
Jp3= cross(T02(1:3,3)',(T03(1:3,4)- T02(1:3,4))');
% Jp1= cross(T01(1:3,3)',(T03(1:3,4)- T01(1:3,4))');
% Jp2= cross(T02(1:3,3)',(T03(1:3,4)- T02(1:3,4))');
% Jp3= cross(T03(1:3,3)',(T03(1:3,4)- T03(1:3,4))');

J=[Jp1(1) Jp2(1) Jp3(1);Jp1(2) Jp2(2) Jp3(2);Jp1(3) Jp2(3) Jp3(3);
    0 sin(q(1)) sin(q(1)); 0 -cos(q(1)) -cos(q(1)); 1 0 0;];

Jp = J(1:3,1:3);

 disp('Jp')
 disp(Jp)
 disp('det')
 disp(det(Jp))
end

