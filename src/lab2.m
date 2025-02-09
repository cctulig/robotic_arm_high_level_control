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

% Calculate cubic coefficients for 4 trajectories for joint 2
traj2A = cubicTrajectory(0,rad(55),0,1,0,0);
traj2B = cubicTrajectory(rad(55),rad(41),1,2,0,0);
traj2C= cubicTrajectory(rad(41),rad(650),2,3,0,0);
traj2D= cubicTrajectory(rad(650),rad(55),3,4,0,0);

% Calculate cubic coefficients for 4 trajectories for joint 3
traj3A = cubicTrajectory(0,rad(-290),0,1,0,0);
traj3B = cubicTrajectory(rad(-290),rad(300),1,2,0,0);
traj3C = cubicTrajectory(rad(300),rad(-350),2,3,0,0);
traj3D = cubicTrajectory(rad(-350),rad(-290),3,4,0,0);

% Interpolate 10 setpoints between each trajectory for joint 2
[interp2A, time2A] = interpolate(traj2A, 0,1);
[interp2B, time2B] = interpolate(traj2B, 1,2);
[interp2C, time2C] = interpolate(traj2C, 2,3);
[interp2D, time2D] = interpolate(traj2D, 3,4);

% Interpolate 10 setpoints between each trajectory for joint 3
[interp3A, ] = interpolate(traj3A, 0,1);
[interp3B, ] = interpolate(traj3B, 1,2);
[interp3C, ] = interpolate(traj3C, 2,3);
[interp3D, ] = interpolate(traj3D, 3,4);

% Combine all 4 trajectories
trajJoint2 = [interp2A interp2B interp2C interp2D];
trajJoint3 = [interp3A interp3B interp3C interp3D];
time = [time2A time2B time2C time2D];

% Assign joint trajectories to queue of setpoints to send to Nucleo
joint2 = trajJoint2; %[55, 41, 650, 55];
joint3 = trajJoint3; %[-290, 300, -350, -290];

% Used for indexing through posiition matrix
index = 1;
i=1;

% We wait to send a new setpoint until the previous has been reached
notReachedSetpoint = 1;


try
    tic
    % Loop through all 40 (10 per traj.) setpoints
    for k = 1:40
        
        DEBUG   = true;          % enables/disables debug prints
        
        
        packet(4) = joint2(k); %Writes setpoint to joint 2
        packet(7) = joint3(k); %Writes setpoint to joint 3
        
        % Send packet to the server and get the response
        %pp.write sends a 15 float packet to the micro controller
        pp.write(SERV_ID, packet);
        
        pause(0.003); % Minimum amount of time required between write and read
        
        %pp.read reads a returned 15 float backet from the nucleo.
        returnPacket = pp.read(SERV_ID);
        
        notReachedSetpoint = 1;
        targetTime = toc + 1;
        while notReachedSetpoint
            
            %Write and reading from the Status server to get encoder positions and motor velocities
            pp.write(STATUS_ID, empty);
            
            pause(0.003); % Minimum amount of time required between write and read
            
            %pp.read reads a returned 15 float backet from the nucleo.
            statusPacket = pp.read(STATUS_ID);
            
            % Convert encoders to radians, then sends those angles to be simulated as a stick model
            angle = [(statusPacket(1)*2*pi/4096), (statusPacket(4)*2*pi/4096), (statusPacket(7)*2*pi/4096)];
            disp(angle)
            stickModel(angle)
            
            % Get the positions of each joint through forward kinematics
            position = fwkin3001(angle(1),angle(2),angle(3));
            matrix = [angle(1),angle(2),angle(3),position(1),position(2),position(3)];
            
            % Record values to the position matrix
            positionMatrix(i,1)= angle(1);
            positionMatrix(i,2)= angle(2);
            positionMatrix(i,3) = angle(3);
            positionMatrix(i,4) = position(1);
            positionMatrix(i,5) = position(2);
            positionMatrix(i,6) = position(3);
            positionMatrix(i,7) = toc;
            
            
            %Once setpoint is roughly reached, move on to the next setpoint
%             if statusPacket(4) < joint2(k) + 10 && statusPacket(4) > joint2(k) - 10
%                 if statusPacket(7) < joint3(k) + 10 && statusPacket(7) > joint3(k) - 10
%                     notReachedSetpoint = 0;
%                 end
%             end

            % Once time interval is reached (0.1s to match interpolation), move on to the next setpoint
            if targetTime < toc
                notReachedSetpoint = 0;
            end
            
            i=i+1;
        end
    end
    
    disp('i: ')
    disp(i)
    
    % Create CSV file of all data recieved from Status Server
    csvwrite('positionMatrix.csv',positionMatrix);
    
    % Graph + format for Joint Angle vs time
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
    plot(positionMatrix(:,7),positionMatrix(:,6),'b', 'LineWidth', 2)
    title('End Effector Position vs. Time');
    set(gca, 'fontsize', 16);
    legend({'X Pos', 'Z Pos'});
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
    
    % Graph + format for X-Position vs. Z-Position (path of end effector)
    figure(5)
    plot(positionMatrix(:,4),positionMatrix(:,6),'r', 'LineWidth', 2)
    grid on
    hold on
    title('X-Position vs. Z-Position ');
    set(gca, 'fontsize', 16);
    xlabel('X Position[mm]'), ylabel('Z Position[mm]');
    hold off
    
    % Graph + format for Planned Trajectory
    figure(6)
    plot(time,zeros(1,40),'r', 'LineWidth', 2)
    grid on
    hold on
    plot(time,trajJoint2,'b', 'LineWidth', 2)
    plot(time,trajJoint3,'g', 'LineWidth', 2)
    title('Planned Trajectory');
    set(gca, 'fontsize', 16);
    legend({'Theta1', 'Theta2', 'Theta3'});
    xlabel('Time[s]'), ylabel('Angle[rad]');
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
zero = [0,0,0];
v1 = [0,0,135];
v2 = link1*link2;
v2 = v2(1:3,4)';
v3 = link1*link2*link3;
v3 = v3(1:3,4);

% Plot the stick model
figure(1)
plot3([0,v1(1),v2(1),v3(1)],[0,v1(2),v2(2), v3(2)],[0,v1(3),v2(3), v3(3)],'b-o')
hold on
xlim([0 400]), ylim([-400 400]), zlim([0 400]);
hold off

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
q = (4096/(2*pi))*val; % Convert angle into an encoder value
end

% Converts encoder values into angles
function [angle] = rad(enc)
angle=enc*((2*pi)/4096);
end

