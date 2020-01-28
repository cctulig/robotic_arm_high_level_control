function [ output_args ] = calibrate( pp )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

for i = 1 : 3
            pause(1);

    returnPacket = zeros(15, 1, 'single');
    resetPacket = zeros(15, 1, 'single');
    
    
    display('I pushed the pushbutton')
    pp.write(03, returnPacket);
    pause(0.003);
    returnPacket = pp.read(03);
    disp(returnPacket);
    
    resetPacket(1) = returnPacket(1);
    resetPacket(2) = returnPacket(4);
    resetPacket(3) = returnPacket(7);
    
    pp.write(04, resetPacket);
    end
end

