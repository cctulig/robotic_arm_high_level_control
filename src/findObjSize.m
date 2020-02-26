function [ BW, sizes ] = findObjSize( img, centroids, cameraParams )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

img = undistortImage(img, cameraParams, 'OutputView', 'full');

F_blur = [1/9 1/9 1/9; 1/9 1/9 1/9; 1/9 1/9 1/9];

img = imfilter(img, F_blur);
[BW,] = createSizeMask(img);
%BW = medfilt2(BW);
se = strel('disk',5,6);
%BW = imdilate(BW, se);
BW = imfill(BW, 'holes');
BW = imerode(BW, se);

s = regionprops(BW, 'centroid', 'FilledArea')

sizes = [0];

[max,] = size(centroids);

for i = 1:max
    for j = 1:size(s,1)
%         disp('Area:')
%         disp(s(j).FilledArea)
%         disp('Centroid:')
%         disp(s(j).Centroid)
        if   s(j).Centroid(1) > centroids(i,1)-30 && s(j).Centroid(1) < centroids(i,1)+30
            disp('x');
            if   s(j).Centroid(2) > centroids(i,2)-30 &&  s(j).Centroid(2) < centroids(i,2)+30
                 disp('y');
                if  s(j).FilledArea > 100
                     disp('hi');
                     sizes(i) = s(j).FilledArea;
                end
            end
        end
    end
end

end

