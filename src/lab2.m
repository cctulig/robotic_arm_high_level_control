%%
% RBE3001 - Laboratory 2
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

disp (vid);
disp (pid);

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
SERV_ID = 01;            % we will be talking to server ID 01 on
% the Nucleo
STATUS_ID = 03;
CALIBRATION_ID = 04;
% Create a PacketProcessor object to send data to the nucleo firmware
pp = PacketProcessor(myHIDSimplePacketComs);
disp('START!')

statusMatrix = zeros(6, 7);
row = 1;

try
    
    
    DEBUG   = true;          % enables/disables debug prints
    
    tic
    while toc < 2
        empty =  zeros(15, 1, 'single');
        
        pp.write(STATUS_ID, empty);
        
        pause(0.003); % Minimum amount of time required between write and read
        
        %pp.read reads a returned 15 float backet from the nucleo.
        statusPacket = pp.read(STATUS_ID);
        for x = 0:2
            
            statusMatrix(row,1+x)= statusPacket((x*3)+1);
            
        end
        statusMatrix(row, 7) = toc;
        
        row = row +1;
    end
    
    csvwrite('status.csv',statusMatrix);
    
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end


% Clear up memory upon termination
pp.shutdown()
