function gripper( position, pp )
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
GRIPPER_ID = 05;       % GripperServer ID on Nucleo

packet(1) = position;
% Send packet to the server and get the response
%pp.write sends a 15 float packet to the micro controller
pp.write(GRIPPER_ID, packet);

pause(0.003); % Minimum amount of time required between write and read

%pp.read reads a returned 15 float backet from the nucleo.
returnPacket = pp.read(GRIPPER_ID);

end

