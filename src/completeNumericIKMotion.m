function completeNumericIKMotion( startPos, endPos, pp, robot )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
SERV_ID = 01; 

angleStart = ikin(startPos(1), startPos(2), startPos(3));
angleEnd = ikin(endPos(1), endPos(2), endPos(3));

notReachedSetpoint = 1;

packet = zeros(15, 1, 'single');
while notReachedSetpoint
    % Calculate Inverse Differential Kinematics
    dq = numericIKAlgo(angleStart, endPos);
    angleStart = angleStart + dq;
    startPos = fwkin3001(angleStart(1), angleStart(2), angleStart(3));
    
    updateRobotModel(angleStart, robot);
    
    packet(1) = convertToEnc(-angleStart(1)); %Writes setpoint to joint 1
    packet(4) = convertToEnc(angleStart(2)); %Writes setpoint to joint 2
    packet(7) = convertToEnc(angleStart(3)); %Writes setpoint to joint 3
    
    % Send packet to the server and get the response
    %pp.write sends a 15 float packet to the micro controller
    pp.write(SERV_ID, packet);
    
    pause(0.003); % Minimum amount of time required between write and read
    
    %pp.read reads a returned 15 float backet from the nucleo.
    returnPacket = pp.read(SERV_ID);
    
    pause(.1)
    
    if startPos(1) > endPos(1) - 1 && startPos(1) < endPos(1) + 1
        if startPos(2) > endPos(2) - 1 && startPos(2) < endPos(2) + 1
            if startPos(3) > endPos(3) - 1 && startPos(3) < endPos(3) + 1
                notReachedSetpoint = false;
                disp('Current Joint Angles:');
                disp(angleStart);
                disp('Target Joint Angles:');
                disp(angleEnd);
            end
        end
    end
end

