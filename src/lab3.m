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
empty = zeros(15, 1, 'single');
positionMatrix=zeros(3, 7);
i=1;

joint1 = [convertToMM(9), convertToMM(3), 200,convertToMM(9), 250,100];
joint2 = [convertToMM(-5), convertToMM(3), 0,convertToMM(-5),20,0];
joint3 = [convertToMM(0), convertToMM(-1), 150,convertToMM(0),40,-10];

setpoints = zeros(4,3);
for k = 1:4
    setpoints(k,:) = ikin(joint1(k), joint2(k), joint3(k));
end

traj2A = quinticTrajectory(convertToMM(9),rad(55),0,1.25,0,0,0,0);
traj2B = quinticTrajectory(convertToMM(3),rad(41),1.25,2.5,0,0,0,0);
traj2C= quinticTrajectory(rad(41),rad(650),2.5,3.75,0,0,0,0);
traj2D= quinticTrajectory(rad(650),rad(55),3.75,5,0,0,0,0);

traj3A = quinticTrajectory(0,rad(-290),0,1.25,0,0,0,0);
traj3B = quinticTrajectory(rad(-290),rad(300),1.25,2.5,0,0,0,0);
traj3C = quinticTrajectory(rad(300),rad(-350),2.5,3.75,0,0,0,0);
traj3D = quinticTrajectory(rad(-350),rad(-290),3.75,5,0,0,0,0);

[interp2A, time2A] = interpolate2(traj2A, 0,1.25);
[interp2B, time2B] = interpolate2(traj2B, 1.25,2.5);
[interp2C, time2C] = interpolate2(traj2C, 2.5,3.75);
[interp2D, time2D] = interpolate2(traj2D, 3.75,5);

[interp3A, ] = interpolate2(traj3A, 0,1.25);
[interp3B, ] = interpolate2(traj3B, 1.25,2.5);
[interp3C, ] = interpolate2(traj3C, 2.5,3.75);
[interp3D, ] = interpolate2(traj3D, 3.75,5);

trajJoint2 = [interp2A interp2B interp2C interp2D];
trajJoint3 = [interp3A interp3B interp3C interp3D];
time = [time2A time2B time2C time2D];

%x 9 y -5 z 0
%x 11 y 0 z 9
%x 3 y 4 z -1
%disp('ikin:')
%target = ikin(180,100,200);
%disp('done')


packet = zeros(15, 1, 'single');
index = 1;
notReachedSetpoint = 1;



try
    tic
    for k = 1:4
        
        %DEBUG   = true;          % enables/disables debug prints
        
        setpoint = ikin(joint1(k), joint2(k), joint3(k));
        %stickModel(setpoint);
        
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
        targetTime = toc + 3;
        while notReachedSetpoint
            
            %Write and reading from the Status server to get encoder positions and motor velocities
            pp.write(STATUS_ID, empty);
            
            pause(0.003); % Minimum amount of time required between write and read
            
            %pp.read reads a returned 15 float backet from the nucleo.
            statusPacket = pp.read(STATUS_ID);
            
            angle = [(statusPacket(1)*2*pi/4096), (statusPacket(4)*2*pi/4096), (statusPacket(7)*2*pi/4096)];
            %disp(angle)
            stickModel(angle)
            position = fwkin3001(angle(1),angle(2),angle(3));
            positionMatrix(i,1)= angle(1);
            matrix = [angle(1),angle(2),angle(3),position(1),position(2),position(3)];
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
            
            if targetTime < toc
                notReachedSetpoint = 0;
            end
            
            i=i+1;
        end
    end
    

    
    csvwrite('positionMatrix.csv',positionMatrix);
    
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
    
    figure(5)
    plot(positionMatrix(:,4),positionMatrix(:,6),'r', 'LineWidth', 2)
    grid on
    hold on
    title('X-Position vs. Z-Position ');
    set(gca, 'fontsize', 16);
    xlabel('X Position[mm]'), ylabel('Z Position[mm]');
    hold off
    
    
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
    
    figure(7)
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

v1 = [0,0,135];
v2 = link1*link2;
v2 = v2(1:3,4)';
v3 = link1*link2*link3;
v3 = v3(1:3,4);
figure(1)
plot3([0,v1(1),v2(1),v3(1)],[0,v1(2),v2(2), v3(2)],[0,v1(3),v2(3), v3(3)],'b-o')
hold on
plot3(v3(1),v3(2),v3(3),'g.-');
xlim([0 400]), ylim([-400 400]), zlim([-50 400]);
hold off

end

function [a] = cubicTrajectory(qi,qf,ti,tf,vi,vf)
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
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
step = (tf-ti)/10;
n=1;
val= zeros(1,10);
timeStamp = zeros(1,10);
for t = ti+step:step:tf
    val(1,n)= a(1)+a(2)*t+a(3)*t^2+a(4)*t^3;
    timeStamp(1,n)= t;
    n=n+1;
end
time = timeStamp;
q = val; %(4096/(2*pi))*val;
end

function [q,time] = interpolate2(a,ti,tf)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
step = (tf-ti)/10;
n=1;
val= zeros(1,10);
timeStamp = zeros(1,10);
for t = ti+step:step:tf
    val(1,n)= a(1)+a(2)*t+a(3)*t^2+a(4)*t^3 + a(5)*t^4 + a(6)*t^5;
    timeStamp(1,n)= t;
    n=n+1;
end
time = timeStamp;
q = val; %(4096/(2*pi))*val;
end

function [angle] = rad(enc)
angle=enc*((2*pi)/4096);
end

function [enc] = convertToEnc(angle)
enc=angle*(4096/(2*pi));
end

function [ theta ] = ikin( x, y, z )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

%task space is 254mm x 304mm x 304mm


l1 = 135;
l2 = 175;
l3 = 169.28;
l4 = sqrt(x^2 + y^2 + (z-l1)^2);
l5 = z-l1;
l6 = sqrt(x^2 + y^2);

theta(1) = atan2(y, x);
disp(theta(1))
theta(2) = (atan2(l5, l6) + acos((l2^2+l4^2-l3^2)/(2*l2*l4)));
disp(theta(2))
theta(3) = -1*((pi/2) - acos((l2^2+l3^2-l4^2)/(2*l2*l3))); 
disp(theta(3))

if((theta(1)> 1.54) || (theta(1)< -1.515) || (theta(2)> 1.725) || (theta(2)< -.14) || (theta(3)> 3.85) || (theta(3)< -.50))
   error('Out of Joint Range'); 
end

if(z< -45 || z > 260 || x < -45 || x > 300 || y < -150 || y > 150)
   error('Out of Bounds'); 
end
    

end

function [millimeters] = convertToMM(squares)
millimeters = squares * 25.4;
end

function [a] = quinticTrajectory(qi,qf,ti,tf,vi,vf,ai,af)
syms a0 a1 a2 a3 a4 a5;

eqns = [a0 + a1*ti + a2*ti^2 + a3*ti^3 + a4*ti^4+a5*ti^5 ==qi, 
        a1 + 2*a2*ti + 3*a3*ti^2 + 4*a4*ti^3 + 5*a5*ti^4 ==vi,
        a0 + a1*tf + a2*tf^2 + a3*tf^3 + a4*tf^4 + a5*tf^5==qf,
        a1 + 2*a2*tf + 3*a3*tf^2 + 4*a4*tf^3 + 5*a5*tf^4== vf,
        2*a2 + 6*a3*ti + 12*a4*ti^2 + 20*a5*ti^3 == ai,
        2*a2 + 6*a3*tf + 12 *a4*tf^2 + 20*a5*tf^3 == af];
    
out = solve(eqns,[a0 a1 a2 a3 a4 a5]);  %'MaxDegree', 3);

a(1)=out.a0;
a(2)=out.a1;
a(3)=out.a2;
a(4)=out.a3;
a(5)=out.a4;
a(6)=out.a5;

end
