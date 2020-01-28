function [ output_args ] = calibrate( pp )
%calibrate Calibrations the home position of the arm
%   When this function is called, ask the status server for the current motor positions,
%   assign them to a new reset packet, and send the reset to the Calibration server
%   to assign the arm's home position to the robot's current position.

for i = 1 : 3
    pause(1); %3 loops with pauses to make sure calibration is sent

    %Initialize packets to be sent/recieved from servers
    returnPacket = zeros(15, 1, 'single');
    resetPacket = zeros(15, 1, 'single');
    
    %Get current encoder values of the arm from the STATUS server
    pp.write(03, returnPacket);
    pause(0.003);
    returnPacket = pp.read(03);
    disp(returnPacket);
    
    %Extract the encoder values from the status packet and put them into the reset packet
    resetPacket(1) = returnPacket(1);
    resetPacket(2) = returnPacket(4);
    resetPacket(3) = returnPacket(7);
    
    %Send the reset packet to the Calibration server
    pp.write(04, resetPacket);
    end
end

