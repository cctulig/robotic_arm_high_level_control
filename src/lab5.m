clear
cam = webcam();
load('cameraParams.mat');
colormap(gray(2))

vid = hex2dec('3742');
pid = hex2dec('0007');

javaaddpath ../lib/SimplePacketComsJavaFat-0.6.4.jar;
import edu.wpi.SimplePacketComs.*;
import edu.wpi.SimplePacketComs.device.*;
import edu.wpi.SimplePacketComs.phy.*;
import java.util.*;
import org.hid4java.*;
import plot_ellipse.*;
import numericIKAlgo.*;
import tb_optparse.*;
version -java
myHIDSimplePacketComs=HIDfactory.get();
myHIDSimplePacketComs.setPid(pid);
myHIDSimplePacketComs.setVid(vid);
myHIDSimplePacketComs.connect();

SERV_ID = 01;          % PidServer ID on Nucleo
STATUS_ID = 03;        % StatusServer ID on Nucleo
CALIBRATION_ID = 04;   % CallibrationServer ID on Nucleo
GRIPPER_ID = 05;       % GripperServer ID on Nucleo

% Create a PacketProcessor object to send data to the nucleo firmware
pp = PacketProcessor(myHIDSimplePacketComs);
packet = zeros(15, 1, 'single');
empty = zeros(15, 1, 'single');

try
    packet(1) = 1;
    % Send packet to the server and get the response
    %pp.write sends a 15 float packet to the micro controller
    pp.write(GRIPPER_ID, packet);
    
    pause(0.003); % Minimum amount of time required between write and read
    
    %pp.read reads a returned 15 float backet from the nucleo.
    returnPacket = pp.read(GRIPPER_ID);
    packet = zeros(15, 1, 'single');
    angleStart = ikin(175, 0, 80);
    
    packet(1) = convertToEnc(angleStart(1)); %Writes setpoint to joint 1
    packet(4) = convertToEnc(angleStart(2)); %Writes setpoint to joint 2
    packet(7) = convertToEnc(angleStart(3)); %Writes setpoint to joint 3
    
    % Send packet to the server and get the response
    %pp.write sends a 15 float packet to the micro controller
    pp.write(SERV_ID, packet);
    
    pause(0.003); % Minimum amount of time required between write and read
    
    %pp.read reads a returned 15 float backet from the nucleo.
    returnPacket = pp.read(SERV_ID);
    
    %pixels = [301, 288; 427, 365; 438, 169; 169, 399;];
    %undistortedPoints = undistortPoints(pixels, cameraParams);
    
    pause(.5);
    
    %actualPoints = [175, 0; 175+2*25.4, 3*25.4; 175-4*25.4, 4*25.4; 175+3*25.4, -3*25.4];
    
    % imwrite(img,'test.png');
    % load('test.png');
    
    img = snapshot(cam);
    
    [centroids, BW] = centroidFinder(img, cameraParams);
    
    [BW2,areas] = findObjSize(img, centroids, cameraParams);
    
    worldPoints = getWorldPoints(centroids, cameraParams);
    
    centroids
    areas
    worldPoints
    
    %Write and reading from the Status server to get encoder positions and motor velocities
    empty = zeros(15, 1, 'single');
    pp.write(STATUS_ID, empty);
    
    pause(0.003); % Minimum amount of time required between write and read
    %
    
    %pp.read reads a returned 15 float backet from the nucleo.
    statusPacket = pp.read(STATUS_ID);
    
    % Convert encoders to radians, then sends those angles to be simulated as a stick model
    %angle = [(statusPacket(1)*2*pi/4096), (statusPacket(4)*2*pi/4096), (statusPacket(7)*2*pi/4096)];
    
    %position = fwkin3001(angle(1),angle(2),angle(3));
    
    startPos = [175, 0, 80];
    completeDynamicNumericIKMotion(startPos, cam, cameraParams, pp);
    gripper(0, pp);
    
%     for i = 1:size(centroids(:,1))
%         startPos = [175, 0, 80];
%         end1Pos = [worldPoints(i,1), worldPoints(i,2), startPos(3)];
%         end2Pos = [worldPoints(i,1), worldPoints(i,2), -20];
%         completeNumericIKMotion(startPos, end1Pos, pp);
%         completeNumericIKMotion(end1Pos, end2Pos, pp);
%         
%         pause(.5);
%         gripper(0, pp);
%         pause(.5);
%         
%         [endX, endY] = determine_Placement(centroids(i,3), areas(1,i));
%         
%         startPos = end2Pos;
%         end1Pos = [startPos(1), startPos(2), 80];
%         end2Pos = [endX, endY, -10];
%         completeNumericIKMotion(startPos, end1Pos, pp);
%         completeNumericIKMotion(end1Pos, end2Pos, pp);
%         
%         pause(.5);
%         gripper(1, pp);
%         pause(.5);
% 
%         
%         startPos = end2Pos;
%         end1Pos = [175, 0, 80];
%         completeNumericIKMotion(startPos, end1Pos, pp);
%     end
    
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

% Record end positions of each trajectory
% % posComp = [posComp; endPos startPos'];
% % notReachedSetpoint = 1;
% %
% % % Get a new mouse input for next target trajectory
% % mouseInput = ginput(1);
% % endPos(1) = mouseInput(1);
% % endPos(3) = mouseInput(2);
% % angleEnd = ikin(xPos(2), yPos(2), zPos(2));


% figure(1)
% colormap(gray(2));
% image(BW2)
% hold on
% plot(centroids(1,1),centroids(1,2),'.r')
% plot(centroids(2,1),centroids(2,2),'.r')
% plot(centroids(3,1),centroids(3,2),'.r')
%
% grid on
% hold on
% title('Black-White Image After Size Filter');
% set(gca, 'fontsize', 16);
% xlabel('X [mm]'), ylabel('Y [mm]');
% hold off

