function [ centroids, BW ] = centroidFinder( frame, cameraParams )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
   img = undistortImage(frame, cameraParams, 'OutputView', 'full');
   I = colouredToGray(frame);
   T = graythresh(I);
   BW = imbinarize(I, T);
   s = regionprops(BW, 'centroid');
    centroids = cat(1, s.Centroid);
end

