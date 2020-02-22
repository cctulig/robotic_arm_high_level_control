function [ centroids, BW ] = centroidFinder( frame, cameraParams )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
   img = undistortImage(frame, cameraParams, 'OutputView', 'full');
   
   F_blur = [1/9 1/9 1/9; 1/9 1/9 1/9; 1/9 1/9 1/9];
   
   BW = imfilter(frame, F_blur);
   [BW,] = createBlueMask(BW);
   se = strel('line',5,5);
   BW = imerode(BW, se);   
%    I = colouredToGray(BW);
%    T = graythresh(I);
%    BW = imbinarize(I, T);

   s = regionprops(BW, 'centroid');
   centroids = cat(1, s.Centroid);
end

