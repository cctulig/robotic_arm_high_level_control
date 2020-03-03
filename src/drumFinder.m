function [ final_centroids, BW  ] = drumFinder(  frame, cameraParams, pp  )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

img = undistortImage(frame, cameraParams, 'OutputView', 'full');

F_blur = [1/9 1/9 1/9; 1/9 1/9 1/9; 1/9 1/9 1/9];

img = imfilter(frame, F_blur);

% BLUE
[BWB,imgB] = createChristmasDrumMask(img);

se = strel('disk',5,6);
BWB = imfill(BWB, 'holes');
BWB = imerode(BWB, se);

s = regionprops(BWB, 'centroid');
blue_cent = cat(1, s.Centroid);

worldPoints = getWorldPoints(blue_cent, cameraParams);


startPos = [175, 0, 80];
end1Pos = [worldPoints(1,1), worldPoints(1,2), startPos(3)];
end2Pos = [worldPoints(1,1), worldPoints(1,2), -20];
completeNumericIKMotion(startPos, end1Pos, pp);
completeNumericIKMotion(end1Pos, end2Pos, pp);

pause(.5);
gripper(0, pp);
pause(.5);

endX = 0;
endY = 220;

startPos = end2Pos;
end1Pos = [startPos(1), startPos(2), 80];
end2Pos = [endX, endY, -10];
completeNumericIKMotion(startPos, end1Pos, pp);
completeNumericIKMotion(end1Pos, end2Pos, pp);

pause(.5);
gripper(1, pp);
pause(.5);


startPos = end2Pos;
end1Pos = [175, 0, 80];
completeNumericIKMotion(startPos, end1Pos, pp);
end

