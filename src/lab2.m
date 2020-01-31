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

n=500;
statusMatrix = zeros(6, 7);
row = 1;
empty = zeros(15, 1, 'single');
positionMatrix=zeros(n, 7);
i=1;

 xPos = [55, 41, 650];
 zPos = [-290, 300, -350];

try
    
    
    DEBUG   = true;          % enables/disables debug prints
    tic
    
    while i < n
           
          %Write and reading from the Status server to get encoder positions and motor velocities
            pp.write(STATUS_ID, empty);
            
            pause(0.003); % Minimum amount of time required between write and read
            
            %pp.read reads a returned 15 float backet from the nucleo.
            statusPacket = pp.read(STATUS_ID);
            
            angle = [(statusPacket(1)*2*pi/4096), (statusPacket(4)*2*pi/4096), (statusPacket(7)*2*pi/4096)];
            disp(angle)
            stickModel(angle)
            position = fwkin3001(angle(1),angle(2),angle(3));
            matrix = [angle(1),angle(2),angle(3),position(1),position(2),position(3)];
            positionMatrix(i,1)= angle(1);
            positionMatrix(i,2)= angle(2);
            positionMatrix(i,3) = angle(3);
            positionMatrix(i,4) = position(1);
            positionMatrix(i,5) = position(2);
            positionMatrix(i,6) = position(3);
            positionMatrix(i,7) = toc;
            i=i+1;
    end
    
    csvwrite('positionMatrix.csv',positionMatrix);
    
    figure(2)
    plot(positionMatrix(:,7),positionMatrix(:,1),'r')
    hold on
    plot(positionMatrix(:,7),positionMatrix(:,2),'b')
    plot(positionMatrix(:,7),positionMatrix(:,3),'g')
    title('Joint Angles');
    set(gca, 'fontsize', 16);
    legend({'Theta1', 'Theta2', 'Theta3'});

    hold off
    
    figure(3)
    plot(positionMatrix(:,7),positionMatrix(:,4),'r')
    hold on
    plot(positionMatrix(:,7),positionMatrix(:,6),'b')
    title('End Effector');
    set(gca, 'fontsize', 16);
    legend({'X Pos', 'Z Pos'});
    hold off
    
    figure(4)
    plot(positionMatrix(1:end-1,7),rdivide(diff(positionMatrix(:,4)'),diff(positionMatrix(:,7)')) ,'r')
    hold on
    plot(positionMatrix(:,7),rdivide(diff(positionMatrix(:,6)'),diff(positionMatrix(:,7)')),'b')
    title('Velocity');
    set(gca, 'fontsize', 16);
    legend({'X Vel', 'Z Vel'});
    hold off
    
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end


% Clear up memory upon termination
pp.shutdown()

function [] = stickModel(q)
%UNTITLED Summa,ry of this function goes here
%   Detailed explanation goes here

link1 = [cos(q(1)), 0, sin(q(1)), 0;
            sin(q(1)), 0, -cos(q(1)), 0;
            0, 1, 0, 135;
            0, 0, 0, 1;];
link2 = [cos(q(2)), -sin(q(2)), 0, 175*cos(q(2));
        sin(q(2)) cos(q(2)), 0, 175*sin(q(2));
        0, 0, 1, 0;
        0, 0, 0, 1;];
link3 = [cos(q(3)-pi/2), -sin(q(3)-pi/2), 0, 169.28*cos(q(3)-pi/2);
        sin(q(3)-pi/2), cos(q(3)-pi/2), 0, 169.28*sin(q(3)-pi/2);
        0, 0, 1, 0;
        0, 0, 0, 1;];
zero = [0,0,0];
    v1 = [0,0,135];
v2 = link1*link2;
v2 = v2(1:3,4)';
v3 = link1*link2*link3;
v3 = v3(1:3,4);
figure(1)
plot3([0,v1(1),v2(1),v3(1)],[0,v1(2),v2(2), v3(2)],[0,v1(3),v2(3), v3(3)],'b-o')
hold on
xlim([0 400]), ylim([-400 400]), zlim([0 400]);
hold off

end


