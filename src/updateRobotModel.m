function updateRobotModel( q, robot )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
config = homeConfiguration(robot);
config(1).JointPosition = q(1);
config(2).JointPosition = q(2);
config(3).JointPosition = q(3);

figure(2);
show(robot,config);

end

