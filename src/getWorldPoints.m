function [ worldPoints ] = getWorldPoints( centroids, cameraParams )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here


T_0CH = [1 0 0 (115+175); 0 1 0 (110-5); 0 0 1 -30; 0 0 0 1];
T_CACH =  [0.0266   -0.8675    0.4968  107.7135;
    0.9995    0.0321    0.0025  101.8156;
    -0.0181    0.4965    0.8679  290.9126;
    0         0         0    1.0000;];

T_0CA = T_0CH * inv(T_CACH);


worldPoints = pointsToWorld(cameraParams, T_CACH(1:3,1:3), T_CACH(1:3,4), centroids(:, 1:2));

worldPoints(:,1) = T_0CH(1,4) - worldPoints(:,1);
worldPoints(:,2) = T_0CH(2,4) + worldPoints(:,2);


end


%     0.0266    0.9995   -0.0181  128.4418
%    -0.8674    0.0321    0.4964   47.3483
%     0.4968    0.0025    0.8678 -336.2277
%          0         0         0    1.0000
