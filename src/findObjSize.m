function [ BW, sizes ] = findObjSize( img, centroids )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

F_blur = [1/9 1/9 1/9; 1/9 1/9 1/9; 1/9 1/9 1/9];


[BW,] = createSizeMask(img);
BW = imfilter(BW, F_blur);
BW = medfilt2(BW);
se = strel('line',10,10);
BW = imdilate(BW, se);
BW = imclose(BW, se);
s = regionprops(BW, 'centroid', 'FilledArea')

sizes = [0];

[max,] = size(centroids)

for i = 1:max
    for j = 1:size(s,1)
        if   s(j).Centroid(1) > centroids(i,1)-30 && s(j).Centroid(1) > centroids(i,1)+30
            if   s(j).Centroid(2) > centroids(i,2)-30 &&  s(j).Centroid(2) > centroids(i,2)+30
                if  s(j).FilledArea > 100
                    sizes(i) = s(j).FilledArea;
                end
            end
        end
    end
end

end

