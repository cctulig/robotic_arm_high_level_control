function [ centroids, BW ] = centroidFinder( frame, cameraParams )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
   img = undistortImage(frame, cameraParams, 'OutputView', 'full');
   
   F_blur = [1/9 1/9 1/9; 1/9 1/9 1/9; 1/9 1/9 1/9];
   
   img = imfilter(frame, F_blur);
   
   % BLUE
   [BWB,imgB] = createBlueMask1(img);
   [BWB,imgB] = createBlueMask2(imgB);
   se = strel('line',5,5);
   BWB = imerode(BWB, se);   

   s = regionprops(BWB, 'centroid');
   blue_cent = cat(1, s.Centroid);
   [sizeB,i] = size(blue_cent);
   blues = ones(sizeB,1);
   blue_cent = [blue_cent, blues];
   
   
   % GREEN
   [BWG,imgG] = createGreenMask1(img);
   [BWG,imgG] = createGreenMask2(imgG);
   se = strel('line',10,10);
   BWG = imerode(BWG, se);   

   s = regionprops(BWG, 'centroid');
   green_cent = cat(1, s.Centroid);
   [sizeG,i] = size(green_cent);
   greens = ones(sizeG,1) + 1;
   green_cent = [green_cent, greens];
   
   % YELLOW
   [BWY,imgY] = createYellowMask1(img);
   [BWY,imgY] = createYellowMask2(imgY);
   se = strel('line',5,5);
   BWY = imerode(BWY, se);   

   s = regionprops(BWY, 'centroid');
   yellow_cent = cat(1, s.Centroid);
   [sizeY,i] = size(yellow_cent);
   yellows = ones(sizeY,1) + 2;
   yellow_cent = [yellow_cent, yellows];
   
   BW = [BWB, BWG, BWY];
   
   centroids = [blue_cent' green_cent' yellow_cent']';
end

