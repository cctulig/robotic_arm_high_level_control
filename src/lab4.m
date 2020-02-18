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

xPos = [convertToMM(9), convertToMM(3), 250,convertToMM(9)];
yPos = [convertToMM(-5), convertToMM(3), 0, convertToMM(-5)];
zPos = [convertToMM(0), convertToMM(-1), 150,convertToMM(0)];

% cubic Interpolation Section:
% Calculate cubic coefficients for 3 trajectories for joint 1
traj1A = cubicTrajectory(xPos(1),xPos(2),0,1.25,0,0);
traj1B = cubicTrajectory(xPos(2),xPos(3),1.25,2.5,0,0);
traj1C= cubicTrajectory(xPos(3),xPos(4),2.5,3.75,0,0);

% Calculate cubic coefficients for 3 trajectories for joint 2
traj2A = cubicTrajectory(yPos(1),yPos(2),0,1.25,0,0);
traj2B = cubicTrajectory(yPos(2),yPos(3),1.25,2.5,0,0);
traj2C= cubicTrajectory(yPos(3),yPos(4),2.5,3.75,0,0);

% Calculate cubic coefficients for 3 trajectories for joint 3
traj3A = cubicTrajectory(zPos(1),zPos(2),0,1.25,0,0);
traj3B = cubicTrajectory(zPos(2),zPos(3),1.25,2.5,0,0);
traj3C = cubicTrajectory(zPos(3),zPos(4),2.5,3.75,0,0);

% Interpolate 10 setpoints between each trajectory for joint 1
[interp1A, ] = interpolate(traj1A, 0,1.25);
[interp1B, ] = interpolate(traj1B, 1.25,2.5);
[interp1C, ] = interpolate(traj1C, 2.5,3.75);

% Interpolate 10 setpoints between each trajectory for joint 2
[interp2A, time2A] = interpolate(traj2A, 0,1.25);
[interp2B, time2B] = interpolate(traj2B, 1.25,2.5);
[interp2C, time2C] = interpolate(traj2C, 2.5,3.75);

% Interpolate 10 setpoints between each trajectory for joint 3
[interp3A, ] = interpolate(traj3A, 0,1.25);
[interp3B, ] = interpolate(traj3B, 1.25,2.5);
[interp3C, ] = interpolate(traj3C, 2.5,3.75);

% Combine all 3 trajectories
trajxPos = [interp1A interp1B interp1C];
trajyPos = [interp2A interp2B interp2C];
trajzPos = [interp3A interp3B interp3C];
time = [time2A time2B time2C];


% We wait to send a new setpoint until the previous has been reached
notReachedSetpoint = 1;
try
    
    %[55, 41, 650, 55];
    %[0,0,0,0]
    %[-290, 300, -350, -290];
    
    startPos = [xPos(1), 0, zPos(1)];
    endPos = [xPos(2), 0, zPos(2)];
    angleStart = ikin(xPos(1), yPos(1), zPos(1));
    angleEnd = ikin(xPos(2), yPos(2), zPos(2));
    
    posComp = [0 0 0 0 0 0];
    %while true
    for i = 1:4
        while notReachedSetpoint
            dq = numericIKAlgo(angleStart, endPos);
            angleStart = angleStart + dq;
            startPos = fwkin3001(angleStart(1), angleStart(2), angleStart(3));
            
            packet(1) = convertToEnc(angleStart(1)); %Writes setpoint to joint 1
            packet(4) = convertToEnc(angleStart(2)); %Writes setpoint to joint 2
            packet(7) = convertToEnc(angleStart(3)); %Writes setpoint to joint 3
            
            % Send packet to the server and get the response
            %pp.write sends a 15 float packet to the micro controller
            pp.write(SERV_ID, packet);
            
            pause(0.003); % Minimum amount of time required between write and read
            
            %pp.read reads a returned 15 float backet from the nucleo.
            returnPacket = pp.read(SERV_ID);
            
            pause(.1);
            
            stickModel(angleStart);
            XZGraph(startPos);
            
            if startPos(1) > endPos(1) - 1 && startPos(1) < endPos(1) + 1
                if startPos(3) > endPos(3) - 1 && startPos(3) < endPos(3) + 1
                    notReachedSetpoint = false;
                    disp('Current Joint Angles:');
                    disp(angleStart);
                    disp('Target Joint Angles:');
                    disp(angleEnd);
                end
            end
        end
        posComp = [posComp; endPos startPos'];
        notReachedSetpoint = 1;
        mouseInput = ginput(1);
        endPos(1) = mouseInput(1);
        endPos(3) = mouseInput(2);
        angleEnd = ikin(xPos(2), yPos(2), zPos(2));
    end
    
    csvwrite('PostionComparision.csv', posComp);
    
    disp('DONE');
    
    notReachedSetpoint = 1;
    
    tic
    % Loop through all 30 (10 per traj.) setpoints
    for k = 1:20
        
        %DEBUG   = true;          % enables/disables debug prints
        
        %Sending cubic Interpolation setpoints to Nucleo
        setpoint = ikin(trajxPos(k), trajyPos(k), trajzPos(k));
        
%         packet(1) = convertToEnc(setpoint(1)); %Writes setpoint to joint 1
%         packet(4) = convertToEnc(setpoint(2)); %Writes setpoint to joint 2
%         packet(7) = convertToEnc(setpoint(3)); %Writes setpoint to joint 3
        
        % Send packet to the server and get the response
        %pp.write sends a 15 float packet to the micro controller
        pp.write(SERV_ID, packet);
        
        pause(0.003); % Minimum amount of time required between write and read
        
        %pp.read reads a returned 15 float backet from the nucleo.
        returnPacket = pp.read(SERV_ID);
        
        notReachedSetpoint = 1;
        targetTime = toc + 15; %0.125
        while notReachedSetpoint
            
            %Write and reading from the Status server to get encoder positions and motor velocities
            pp.write(STATUS_ID, empty);
            
            pause(0.003); % Minimum amount of time required between write and read
            %
            
            %pp.read reads a returned 15 float backet from the nucleo.
            statusPacket = pp.read(STATUS_ID);
            
            % Convert encoders to radians, then sends those angles to be simulated as a stick model
            angle = [(statusPacket(1)*2*pi/4096), (statusPacket(4)*2*pi/4096), (statusPacket(7)*2*pi/4096)];
            
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
            
            %Jacobian stuff
            if i>1
                dQ(1)=(positionMatrix(i,1)-positionMatrix(i-1,1))/(positionMatrix(i,7)-positionMatrix(i-1,7));
                dQ(2)=(positionMatrix(i,2)-positionMatrix(i-1,2))/(positionMatrix(i,7)-positionMatrix(i-1,7));
                dQ(3)=(positionMatrix(i,3)-positionMatrix(i-1,3))/(positionMatrix(i,7)-positionMatrix(i-1,7));
                
                jacobian=jacob0(angle);
                jP= jacobian(1:3,1:3);
                
                dP = positionVelocity(angle, dQ);
                stickModel2(angle,dP,jP)
                
            end
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

function [angle] = rad(enc)
angle=enc*((2*pi)/4096);
end

% Converts encoder values into angles
function [enc] = convertToEnc(angle)
enc=angle*(4096/(2*pi));
end

function [ theta ] = ikin( x, y, z )
%ikin Inverse kinematices of a given (x,y,z) setpoint
%   Uses a geometric approach to calculate theta1, theta2, theta3 based off the x,y,z inputs
%

%task space is 254mm x 304mm x 304mm

% Joint and 'imaginary' joint lengths
l1 = 135;
l2 = 175;
l3 = 169.28;
l4 = sqrt(x^2 + y^2 + (z-l1)^2);
l5 = z-l1;
l6 = sqrt(x^2 + y^2);

% Calculating theta1,2,3
theta(1) = atan2(y, x);
%disp(theta(1))
theta(2) = (atan2(l5, l6) + acos((l2^2+l4^2-l3^2)/(2*l2*l4)));
%disp(theta(2))
theta(3) = -1*((pi/2) - acos((l2^2+l3^2-l4^2)/(2*l2*l3)));
%disp(theta(3))

% Error catch for angles that are outside the motor turning range
if((theta(1)> 1.54) || (theta(1)< -1.515) || (theta(2)> 1.725) || (theta(2)< -.14) || (theta(3)> 3.85) || (theta(3)< -.55))
    error('Out of Joint Range');
end

% Error catch for positions that are outside the motor's reach
if(z< -45 || z > 260 || x < -45 || x > 300 || y < -150 || y > 150)
    disp(x);
    disp(y);
    disp(z);
    error('Out of Bounds');
end

end

% Converts inches to millimeters
function [millimeters] = convertToMM(squares)
millimeters = squares * 25.4;
end

function [a] = cubicTrajectory(qi,qf,ti,tf,vi,vf)
%cubicTrajectory Calculates cubic coefficients for a joint trajectory
%   Solves a system of equations to find the cubic coefficients
syms a0 a1 a2 a3;

eqns = [a0 + a1*ti + a2*ti^2 + a3*ti^3 == qi,
    a1 + 2*a2*ti + 3*a3*ti^2 == vi,
    a0 + a1*tf + a2*tf^2 + a3*tf^3 == qf,
    a1 + 2*a2*tf + 3*a3*tf^2 == vf];

out = solve(eqns,[a0 a1 a2 a3]);  %'MaxDegree', 3);

a(1)=out.a0;
a(2)=out.a1;
a(3)=out.a2;
a(4)=out.a3;
end


function [q,time] = interpolate(a,ti,tf)
%interpolate Interpolate 10 setpoints for a given trajectory
%   Generate 10 evenly spaced time stamps between the initial and final times
%   and then calcultes each setpoint based off the inputted cubic coefficients

step = (tf-ti)/10; % Step size
n=1;

% Initialize position and time arrays
val= zeros(1,10);
timeStamp = zeros(1,10);

% Step through each timestamp, calculating angle position for each
for t = ti+step:step:tf
    val(1,n)= a(1)+a(2)*t+a(3)*t^2+a(4)*t^3;
    timeStamp(1,n)= t;
    n=n+1;
end
time = timeStamp;
q = val; % Convert angle into an encoder value
end

function [] = stickModel(q)
%stickModel simulates stick model of arm
%   Uses variable joint angles and define joint lengths to
%   calculate and plot the three joints of the Robotic Arm in MATLAB

% Matrix of each joint transformation
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

% Vectors for each joint
v1 = [0,0,135];
v2 = link1*link2;
v2 = v2(1:3,4)';
v3 = link1*link2*link3;
v3 = v3(1:3,4);

% Plot the stick model
figure(1)
plot3([0,v1(1),v2(1),v3(1)],[0,v1(2),v2(2), v3(2)],[0,v1(3),v2(3), v3(3)],'b-o')
hold on
plot3(v3(1),v3(2),v3(3),'g.-');
xlim([0 400]), ylim([-400 400]), zlim([-50 400]);
hold off

end

function [] = stickModel2(q, dP,jP)
%stickModel2 simulates stick model of arm, with a velocity vector on the
%end effector
%   Uses variable joint angles and define joint lengths to
%   calculate and plot the three joints of the Robotic Arm in MATLAB

% Matrix of each joint transformation
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

% Vectors for each joint
v1 = [0,0,135];
v2 = link1*link2;
v2 = v2(1:3,4)';
v3 = link1*link2*link3;
v3 = v3(1:3,4);

% Plot the stick model
figure(2)
plot3([0,v1(1),v2(1),v3(1)],[0,v1(2),v2(2), v3(2)],[0,v1(3),v2(3), v3(3)],'b-o')
hold on
plot3(v3(1),v3(2),v3(3),'g.-');
xlim([-400 800]), ylim([-800 800]), zlim([-100 800]);
quiver3(v3(1),v3(2),v3(3), dP(1),dP(2),dP(3));
%Vol = plot_ellipse(jP*jP',v3);

title('End Effector Position');
set(gca, 'fontsize', 16);
xlabel('X Position[mm]'), ylabel('Y Position[mm]'), zlabel('Z Position[mm]');

%disp('Vol')
%disp(Vol);

% if Vol < 1000000
%     text(200,600,600,'NEARING SINGULARITY', 'Color', 'r', 'FontSize', 11);
%     
%     %error('NEARING SINGULARITY');
% end

hold off

end

function [ dP ] = positionVelocity( q, dQ )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
J=jacob0(q);
dP=J*dQ';
end
function [ J ] = jacob0( q )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

l1 = 135;
l2 = 175;
l3 = 169.28;

T01 = [cos(q(1)), 0, sin(q(1)), 0;
    sin(q(1)), 0, -cos(q(1)), 0;
    0, 1, 0, l1;
    0, 0, 0, 1;];
T12 = [cos(q(2)), -sin(q(2)), 0, l2*cos(q(2));
    sin(q(2)) cos(q(2)), 0, l2*sin(q(2));
    0, 0, 1, 0;
    0, 0, 0, 1;];
T23 = [sin(q(3)), cos(q(3)), 0, l3*sin(q(3));
    -cos(q(3)), sin(q(3)), 0, -1*l3*cos(q(3));
    0, 0, 1, 0;
    0, 0, 0, 1;];
T02= T01*T12;
% disp('T02')
% disp(T02)
T03=T02*T23;
% disp('T03')
% disp(T03)
Jp1= cross([0 0 1],T03(1:3,4));
Jp2= cross(T01(1:3,3)',(T03(1:3,4)- T01(1:3,4))');
Jp3= cross(T02(1:3,3)',(T03(1:3,4)- T02(1:3,4))');
% Jp1= cross(T01(1:3,3)',(T03(1:3,4)- T01(1:3,4))');
% Jp2= cross(T02(1:3,3)',(T03(1:3,4)- T02(1:3,4))');
% Jp3= cross(T03(1:3,3)',(T03(1:3,4)- T03(1:3,4))');

J=[Jp1(1) Jp2(1) Jp3(1);Jp1(2) Jp2(2) Jp3(2);Jp1(3) Jp2(3) Jp3(3);
    0 sin(q(1)) sin(q(1)); 0 -cos(q(1)) -cos(q(1)); 1 0 0;];

% disp('J')
% disp(J)
% disp('det')
% disp(det(J))
end

function [] = XZGraph (p)

% Graph + format for X-Position vs. Z-Position (path of end effector)
figure(5)
plot(p(1),p(3),'.r')
grid on
hold on
title('X-Position vs. Z-Position ');
set(gca, 'fontsize', 16);
xlabel('X Position[mm]'), ylabel('Z Position[mm]');
xlim([0 300]), ylim([-30 100]);

end

