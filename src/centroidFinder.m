function [ final_centroids, BW ] = centroidFinder( frame, cameraParams )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
img = undistortImage(frame, cameraParams, 'OutputView', 'full');

F_blur = [1/9 1/9 1/9; 1/9 1/9 1/9; 1/9 1/9 1/9];

img = imfilter(frame, F_blur);

% BLUE
[BWB,imgB] = createBlueMask(img);

se = strel('disk',5,6);
BWB = imfill(BWB, 'holes');
BWB = imerode(BWB, se);

s = regionprops(BWB, 'centroid');
blue_cent = cat(1, s.Centroid);
[sizeB,i] = size(blue_cent);
blues = ones(sizeB,1);
blue_cent = [blue_cent, blues];

% GREEN
[BWG,imgG] = createGreenMask(img);
se = strel('disk',5,8);
BWG = imfill(BWG, 'holes');
BWG = imerode(BWG, se);

s = regionprops(BWG, 'centroid');
green_cent = cat(1, s.Centroid);
[sizeG,i] = size(green_cent);
greens = ones(sizeG,1) + 1;
green_cent = [green_cent, greens];

% YELLOW
[BWY,imgY] = createYellowMask(img);
se = strel('disk',5,6);
BWY = imfill(BWY, 'holes');
BWY = imerode(BWY, se);

s = regionprops(BWY, 'centroid');
yellow_cent = cat(1, s.Centroid);
[sizeY,i] = size(yellow_cent);
yellows = ones(sizeY,1) + 2;
yellow_cent = [yellow_cent, yellows];

BW = BWY;

centroids = [blue_cent' green_cent' yellow_cent']';

[max,] = size(centroids);

% if max > 1
%     index = 2;
%     while index <= max
%         if centroids(index, 1) > centroids(index-1, 1)-50 && centroids(index, 1) < centroids(index-1, 1)+50
%             if centroids(index, 2) > centroids(index-1, 2)-50 && centroids(index, 2) < centroids(index-1, 2)+50
%                 if centroids(index, 3) == centroids(index-1, 3)
%                     centroids(index,:) = [];
%                     index = index-1;
%                     max=max-1;
%                 end
%             end
%         end
%         index = index+1;
%     end
% end

final_centroids = centroids;
end

