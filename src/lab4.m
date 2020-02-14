%%
% RBE3001 - Laboratory 4
%
% Instructions
% ------------
% Welcome again! This MATLAB script is your starting point for Lab
% 1 of RBE3001. The sample code below demonstrates how to establish
% communication between this script and the Nucleo firmware, send
% setpoint commands and receive sensor data.
%
% IMPORTANT - understanding the code below requires being familiar
% with the Nucleo firmware. Read that code first.

% Lines 15-37 perform necessary library initializations. You can skip reading
% to line 38.
clear
clear java
clear classes;

vid = hex2dec('3742');
pid = hex2dec('0007');

%disp (vid);
%disp (pid);

javaaddpath ../lib/SimplePacketComsJavaFat-0.6.4.jar;
import edu.wpi.SimplePacketComs.*;
import edu.wpi.SimplePacketComs.device.*;
import edu.wpi.SimplePacketComs.phy.*;
import java.util.*;
import org.hid4java.*;
version -java
myHIDSimplePacketComs=HIDfactory.get();
myHIDSimplePacketComs.setPid(pid);
myHIDSimplePacketComs.setVid(vid);
myHIDSimplePacketComs.connect();

SERV_ID = 01;          % PidServer ID on Nucleo
STATUS_ID = 03;        % StatusServer ID on Nucleo
CALIBRATION_ID = 04;   % CallibrationServer ID on Nucleo

% Create a PacketProcessor object to send data to the nucleo firmware
pp = PacketProcessor(myHIDSimplePacketComs);

disp('START!')

% Initialize Matrices to zero
statusMatrix = zeros(6, 7);
empty = zeros(15, 1, 'single');
positionMatrix=zeros(3, 7);
packet = zeros(15, 1, 'single');

% Used for indexing through posiition matrix
index = 1;
i=1;

% We wait to send a new setpoint until the previous has been reached
notReachedSetpoint = 1;
try
    tic
    % Loop through all 30 (10 per traj.) setpoints
    for k = 1:3
        
        %DEBUG   = true;          % enables/disables debug prints
        
        % Sending Linear Interpolate setpoints to Nucleo
        %         setpoint = ikin(linTraj(1,k), linTraj(2,k), linTraj(3,k));
        %         %stickModel(setpoint);
        %
        %         packet(1) = convertToEnc(setpoint(1));
        %         packet(4) = convertToEnc(setpoint(2));
        %         packet(7) = convertToEnc(setpoint(3));
        
        %Sending Quintic Interpolation setpoints to Nucleo
        % setpoint = ikin(trajxPos(k), trajyPos(k), trajzPos(k));
        
       % packet(1) = convertToEnc(setpoint(1)); %Writes setpoint to joint 1
       % packet(4) = convertToEnc(setpoint(2)); %Writes setpoint to joint 2
       % packet(7) = convertToEnc(setpoint(3)); %Writes setpoint to joint 3
                
        % Send packet to the server and get the response
        %pp.write sends a 15 float packet to the micro controller
        pp.write(SERV_ID, packet);
        
        pause(0.003); % Minimum amount of time required between write and read
        
        %pp.read reads a returned 15 float backet from the nucleo.
        returnPacket = pp.read(SERV_ID);
        
        notReachedSetpoint = 1;
        targetTime = toc + 2;
        while notReachedSetpoint
            
            %Write and reading from the Status server to get encoder positions and motor velocities
            pp.write(STATUS_ID, empty);
            
            pause(0.003); % Minimum amount of time required between write and read
            %     
     
            %pp.read reads a returned 15 float backet from the nucleo.
            statusPacket = pp.read(STATUS_ID);
            
            % Convert encoders to radians, then sends those angles to be simulated as a stick model
            angle = [(statusPacket(1)*2*pi/4096), (statusPacket(4)*2*pi/4096), (statusPacket(7)*2*pi/4096)]
            stickModel(angle)
            position = fwkin3001(angle(1),angle(2),angle(3));
            
            % Get the positions of each joint through forward kinematics
            positionMatrix(i,1)= angle(1);
            matrix = [angle(1),angle(2),angle(3),position(1),position(2),position(3)];
            
            % Record values to the position matrix
            positionMatrix(i,2)= angle(2);
            positionMatrix(i,3) = angle(3);
            positionMatrix(i,4) = position(1);
            positionMatrix(i,5) = position(2);
            positionMatrix(i,6) = position(3);
            positionMatrix(i,7) = toc;
            
            % Once setpoint is reached, go to the next setpoint
            if targetTime < toc
                notReachedSetpoint = 0;
            end
            
            i=i+1;
        end
    end
      
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end