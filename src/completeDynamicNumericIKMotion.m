function completeDynamicNumericIKMotion( startPos, cam, cameraParams, pp )
%UNTITLED19 Summary of this function goes here
%   Detailed explanation goes here
SERV_ID = 01;

angleStart = ikin(startPos(1), startPos(2), startPos(3));

notReachedSetpoint = 1;

packet = zeros(15, 1, 'single');
while notReachedSetpoint
    
    img = snapshot(cam);
    [centroids, BW] = centroidFinder(img, cameraParams);
    
    if size(centroids) > 0
        worldPoints = getWorldPoints(centroids, cameraParams);
    end
    
    endPos = [worldPoints(1,1), worldPoints(1,2), 60];
    if startPos(1) > endPos(1) - 3 && startPos(1) < endPos(1) + 3
        if startPos(2) > endPos(2) - 3 && startPos(2) < endPos(2) + 3
            endPos = [worldPoints(1,1), worldPoints(1,2), -20];
            if startPos(3) > endPos(3) - 1 && startPos(3) < endPos(3) + 1
                notReachedSetpoint = false;
            end
        end
    end
    angleEnd = ikin(endPos(1), endPos(2), endPos(3));
    
    
    
    % Calculate Inverse Differential Kinematics
    dq = numericIKAlgo(angleStart, endPos);
    angleStart = angleStart + dq;
    startPos = fwkin3001(angleStart(1), angleStart(2), angleStart(3));
    
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
    
end

