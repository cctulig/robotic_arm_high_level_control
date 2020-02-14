%%
% RBE3001 - Laboratory 3
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

% Setpoints used for arbitrary trajectory for the bonus
% which we chose to create two sinusoidal wave motions
%  yPos = zeros(300,1);T01 = [cos(q(1)), 0, sin(q(1)), 0;
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
%  zPos = zeros(300,1);
%  xPos = zeros(300,1);
% 
% for y = 1:300
%    yPos(y) = y - 150;
%    zPos(y) = 75*sin(2*pi*y/250) + 60;
%    xPos(y) = 25*sin(2*pi*y/100)+180;
% end

% Triangular Trajectory Setpoints
% Note: convertToMM converts inches to mm since the robot task space is gridded with 1 inch blocks
xPos = [convertToMM(9), convertToMM(3), 250,convertToMM(9)];
yPos = [convertToMM(-5), convertToMM(3), 0, convertToMM(-5)];
zPos = [convertToMM(0), convertToMM(-1), 150,convertToMM(0)];

% Linear Interpolation Section:
lin1=linInterpolate([xPos(1) yPos(1) zPos(1)],[xPos(2) yPos(2) zPos(2)]);
lin2=linInterpolate([xPos(2) yPos(2) zPos(2)],[xPos(3) yPos(3) zPos(3)]);
lin3=linInterpolate([xPos(3) yPos(3) zPos(3)],[xPos(4) yPos(4) zPos(4)]);

linTraj = [lin1, lin2, lin3];

% Quintic Interpolation Section:
% Calculate quintic coefficients for 3 trajectories for joint 1
traj1A = quinticTrajectory(xPos(1),xPos(2),0,1.25,0,0,0,0);
traj1B = quinticTrajectory(xPos(2),xPos(3),1.25,2.5,0,0,0,0);
traj1C= quinticTrajectory(xPos(3),xPos(4),2.5,3.75,0,0,0,0);

% Calculate quintic coefficients for 3 trajectories for joint 2
traj2A = quinticTrajectory(yPos(1),yPos(2),0,1.25,0,0,0,0);
traj2B = quinticTrajectory(yPos(2),yPos(3),1.25,2.5,0,0,0,0);
traj2C= quinticTrajectory(yPos(3),yPos(4),2.5,3.75,0,0,0,0);

% Calculate quintic coefficients for 3 trajectories for joint 3
traj3A = quinticTrajectory(zPos(1),zPos(2),0,1.25,0,0,0,0);
traj3B = quinticTrajectory(zPos(2),zPos(3),1.25,2.5,0,0,0,0);
traj3C = quinticTrajectory(zPos(3),zPos(4),2.5,3.75,0,0,0,0);

% Interpolate 10 setpoints between each trajectory for joint 1
[interp1A, ] = interpolate2(traj1A, 0,1.25);
[interp1B, ] = interpolate2(traj1B, 1.25,2.5);
[interp1C, ] = interpolate2(traj1C, 2.5,3.75);

% Interpolate 10 setpoints between each trajectory for joint 2
[interp2A, time2A] = interpolate2(traj2A, 0,1.25);
[interp2B, time2B] = interpolate2(traj2B, 1.25,2.5);
[interp2C, time2C] = interpolate2(traj2C, 2.5,3.75);

% Interpolate 10 setpoints between each trajectory for joint 3
[interp3A, ] = interpolate2(traj3A, 0,1.25);
[interp3B, ] = interpolate2(traj3B, 1.25,2.5);
[interp3C, ] = interpolate2(traj3C, 2.5,3.75);

% Combine all 3 trajectories
trajxPos = [interp1A interp1B interp1C];
trajyPos = [interp2A interp2B interp2C];
trajzPos = [interp3A interp3B interp3C];
time = [time2A time2B time2C];

% Assign joint trajectories to queue of setpoints to send to Nucleo
setpoints = zeros(4,3);
for k = 1:4
    setpoints(k,:) = ikin(trajxPos(k), trajyPos(k), trajzPos(k));
end

% Used for indexing through posiition matrix
index = 1;
i=1;

% We wait to send a new setpoint until the previous has been reached
notReachedSetpoint = 1;

try
    % Determine the average response time of Status Server
    roundTrip = zeros(500, 1);
    for l = 1:500
        tic
        pp.write(STATUS_ID, empty);   
        pause(0.003); % Minimum amount of time required between write and read       
        %pp.read reads a returned 15 float backet from the nucleo.
        statusPacket = pp.read(STATUS_ID);
        roundTrip(l) = toc;
    end
    
    % Creates histogram of response times of Status Server
    figure(12);
    histogram(roundTrip);
    hold on
    title('Histogram of Status Server Response Time');
    set(gca, 'fontsize', 16);
    xlabel('Time[s]');
    hold off
    
    % Print out std, mean, max, min of response times
    fprintf("std: %f\n",std(roundTrip))
    fprintf("mean: %f\n", mean(roundTrip))
    fprintf("min: %f\n", min(roundTrip))
    fprintf("max: %f\n", max(roundTrip))
    
    tic
    % Loop through all 30 (10 per traj.) setpoints
    for k = 1:30
        
        %DEBUG   = true;          % enables/disables debug prints
        
        % Sending Linear Interpolate setpoints to Nucleo
        %         setpoint = ikin(linTraj(1,k), linTraj(2,k), linTraj(3,k));
        %         %stickModel(setpoint);
        %
        %         packet(1) = convertToEnc(setpoint(1));
        %         packet(4) = convertToEnc(setpoint(2));
        %         packet(7) = convertToEnc(setpoint(3));
        
        %Sending Quintic Interpolation setpoints to Nucleo
        setpoint = ikin(trajxPos(k), trajyPos(k), trajzPos(k));
        
        packet(1) = convertToEnc(setpoint(1)); %Writes setpoint to joint 1
        packet(4) = convertToEnc(setpoint(2)); %Writes setpoint to joint 2
        packet(7) = convertToEnc(setpoint(3)); %Writes setpoint to joint 3
                
        % Send packet to the server and get the response
        %pp.write sends a 15 float packet to the micro controller
        pp.write(SERV_ID, packet);
        
        pause(0.003); % Minimum amount of time required between write and read
        
        %pp.read reads a returned 15 float backet from the nucleo.
        returnPacket = pp.read(SERV_ID);
        
        notReachedSetpoint = 1;
        targetTime = toc + .125;
        while notReachedSetpoint
            
            %Write and reading from the Status server to get encoder positions and motor velocities
            pp.write(STATUS_ID, empty);
            
            pause(0.003); % Minimum amount of time required between write and read
            %     
     
            % Graph and format for the theoretical planned trajectory
            figure(6)
            plot(time,trajxPos,'r', 'LineWidth', 2)
            grid on
            hold on
            plot(time,trajyPos,'b', 'LineWidth', 2)
            plot(time,trajzPos,'g', 'LineWidth', 2)
            title('Planned Trajectory');
            set(gca, 'fontsize', 16);
            legend({'Theta1', 'Theta2', 'Theta3'});
            xlabel('Time[s]'), ylabel('Angle[rad]');
            hold off
     
            %pp.read reads a returned 15 float backet from the nucleo.
            statusPacket = pp.read(STATUS_ID);
            
            % Convert encoders to radians, then sends those angles to be simulated as a stick model
            angle = [(statusPacket(1)*2*pi/4096), (statusPacket(4)*2*pi/4096), (statusPacket(7)*2*pi/4096)];
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
    
    % Create CSV file of all data recieved from Status Server
    csvwrite('positionMatrix.csv',positionMatrix);
    
    % Graph + format for Joint Angles vs time
    figure(2)
    plot(positionMatrix(:,7),positionMatrix(:,1),'r', 'LineWidth', 2)
    grid on
    hold on
    plot(positionMatrix(:,7),positionMatrix(:,2),'b', 'LineWidth', 2)
    plot(positionMatrix(:,7),positionMatrix(:,3),'g', 'LineWidth', 2)
    title('Joint Angle vs. Time');
    set(gca, 'fontsize', 16);
    legend({'Theta1', 'Theta2', 'Theta3'});
    xlabel('Time[s]'), ylabel('Angle[rad]');
    hold off
    
    % Graph + format for End Effector Position vs Time
    figure(3)
    plot(positionMatrix(:,7),positionMatrix(:,4),'r', 'LineWidth', 2)
    grid on
    hold on
    plot(positionMatrix(:,7),positionMatrix(:,5),'g', 'LineWidth', 2)
    plot(positionMatrix(:,7),positionMatrix(:,6),'b', 'LineWidth', 2)
    title('End Effector Position vs. Time');
    set(gca, 'fontsize', 16);
    legend({'X Pos', 'Y Pos','Z Pos'});
    xlabel('Time[s]'), ylabel('Position[mm]');
    hold off
    
    % Graph + format for Joint Velocity vs Time
    figure(4)
    plot(positionMatrix(1:end-1,7),rdivide(diff(positionMatrix(:,1)'),diff(positionMatrix(:,7)')) ,'r', 'LineWidth', 2)
    grid on
    hold on
    plot(positionMatrix(1:end-1,7),rdivide(diff(positionMatrix(:,2)'),diff(positionMatrix(:,7)')),'b', 'LineWidth', 2)
    plot(positionMatrix(1:end-1,7),rdivide(diff(positionMatrix(:,3)'),diff(positionMatrix(:,7)')),'g', 'LineWidth',2)
    title('Joint Velocity vs. Time');
    set(gca, 'fontsize', 16);
    legend({'Joint 1', 'Joint 2','Joint 3'});
    xlabel('Time[s]'), ylabel('Velocity[rad/s]');
    hold off
    
%     
%     
%     figure(6)
%     plot(time,trajxPos,'r', 'LineWidth', 2)
%     grid on
%     hold on
%     plot(time,trajyPos,'b', 'LineWidth', 2)
%     plot(time,trajzPos,'g', 'LineWidth', 2)
%     title('Planned Trajectory');
%     set(gca, 'fontsize', 16);
%     legend({'Theta1', 'Theta2', 'Theta3'});
%     xlabel('Time[s]'), ylabel('Angle[rad]');
%     hold off
%     
    % Graph + format for Joint Accelerations vs Time
    figure(7)
    plot(positionMatrix(1:end-2,7),rdivide(diff(positionMatrix(:,1)',2),diff(positionMatrix(:,7)',2)) ,'r', 'LineWidth', 2)
    grid on
    hold on
    plot(positionMatrix(1:end-2,7),rdivide(diff(positionMatrix(:,2)',2),diff(positionMatrix(:,7)',2)),'b', 'LineWidth', 2)
    plot(positionMatrix(1:end-2,7),rdivide(diff(positionMatrix(:,3)',2),diff(positionMatrix(:,7)',2)),'g', 'LineWidth',2)
    title('Joint Acceleration vs. Time');
    set(gca, 'fontsize', 16);
    legend({'Joint 1', 'Joint 2','Joint 3'});
    xlabel('Time[s]'), ylabel('Accelertation[rad/s^2]');
    hold off
    
    % Graph + format of end effector path in task space
    figure(8)
    plot3(positionMatrix(:,4),positionMatrix(:,5),positionMatrix(:,6),'g.-');
    hold on
    title('End Effector Path in Task Space');
    xlim([0 400]), ylim([-400 400]), zlim([-50 400]);
    xlabel('X Position (mm)'), ylabel('Y Position (mm)'),zlabel('Z Position (mm)');
    hold off
    
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end


% Clear up memory upon termination
pp.shutdown()


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

function [q,time] = interpolate2(a,ti,tf)
%interpolate Interpolate 10 setpoints for a given trajectory
%   Generate 10 evenly spaced time stamps between the initial and final times
%   and then calcultes each setpoint based off the inputted quintic coefficients

step = (tf-ti)/10; % Step size
n=1;

% Initialize position and time arrays
val= zeros(1,10);
timeStamp = zeros(1,10);

% Step through each timestamp, calculating angle position for each
for t = ti+step:step:tf
    val(1,n)= a(1)+a(2)*t+a(3)*t^2+a(4)*t^3 + a(5)*t^4 + a(6)*t^5;
    timeStamp(1,n)= t;
    n=n+1;
end
time = timeStamp;
q = val; 
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
    error('Out of Bounds');
end

end

% Converts inches to millimeters
function [millimeters] = convertToMM(squares)
millimeters = squares * 25.4;
end

function [a] = quinticTrajectory(qi,qf,ti,tf,vi,vf,ai,af)
%cubicTrajectory Calculates quintic coefficients for a joint trajectory
%   Solves a system of equations to find the quintic coefficients
syms a0 a1 a2 a3 a4 a5;

% System of equations for qi/f, vi/f, and ai/f
eqns = [a0 + a1*ti + a2*ti^2 + a3*ti^3 + a4*ti^4+a5*ti^5 ==qi,
    a1 + 2*a2*ti + 3*a3*ti^2 + 4*a4*ti^3 + 5*a5*ti^4 ==vi,
    a0 + a1*tf + a2*tf^2 + a3*tf^3 + a4*tf^4 + a5*tf^5==qf,
    a1 + 2*a2*tf + 3*a3*tf^2 + 4*a4*tf^3 + 5*a5*tf^4== vf,
    2*a2 + 6*a3*ti + 12*a4*ti^2 + 20*a5*ti^3 == ai,
    2*a2 + 6*a3*tf + 12 *a4*tf^2 + 20*a5*tf^3 == af];

% Solving system of equations for quintic coefficients
out = solve(eqns,[a0 a1 a2 a3 a4 a5]);  %'MaxDegree', 3);

a(1)=out.a0;
a(2)=out.a1;
a(3)=out.a2;
a(4)=out.a3;
a(5)=out.a4;
a(6)=out.a5;

end

function[pos] = linInterpolate(pi,pf)
%linInterpolate Interpolates 10 setpoints for a given linear trajectory
%   Generate 10 evenly spaced time stamps between the initial and final times
%   and then calcultes each setpoint based on each interval

% Initialize position array
pos=zeros(3,10);

% Step sizes
stepx= (pf(1)-pi(1))/10;
stepy= (pf(2)-pi(2))/10;
stepz= (pf(3)-pi(3))/10;

% Step through each timestamp, calculating angle position for each
for t = 1:10
    pos(1,t)= pi(1)+stepx*t;
    pos(2,t)= pi(2)+stepy*t;
    pos(3,t)= pi(3)+stepz*t;
end
end
