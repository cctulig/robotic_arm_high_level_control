function [ x, y ] = determine_Placement( color, size )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

y = 220;
if size > 5000
    y = -220;
end

if color == 1
    x = 0;
    
elseif color == 2
    x = 80;
else
    x = 160;
end

end

