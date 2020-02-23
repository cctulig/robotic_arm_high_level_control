

%getCamToCheckerboard(cam, cameraParams)




T_0CH = [1 0 0 (52.8+175); 0 1 0 101.6; 0 0 1 -30; 0 0 0 1];
T_CACH =  [0.0266   -0.8675    0.4968  107.7135;
    0.9995    0.0321    0.0025  101.8156;
    -0.0181    0.4965    0.8679  290.9126;
    0         0         0    1.0000;];

T_0CA = T_0CH * inv(T_CACH);

pixels = [301, 288; 427, 365; 438, 169; 169, 399;];
undistortedPoints = undistortPoints(pixels, cameraParams);

worldPoints = pointsToWorld(cameraParams, T_CACH(1:3,1:3), T_CACH(1:3,4), pixels);

worldPoints(:,1) = T_0CH(1,4) - worldPoints(:,1);
worldPoints(:,2) = T_0CH(2,4) + worldPoints(:,2);

actualPoints = [175, 0; 175+2*25.4, 3*25.4; 175-4*25.4, 4*25.4; 175+3*25.4, -3*25.4];

img = snapshot(cam);
% imwrite(img,'test.png');
% load('test.png');

[centroids, BW] = centroidFinder(img, cameraParams);

[BW2,areas] = findObjSize(img, centroids, cameraParams);
%     0.0266    0.9995   -0.0181  128.4418
%    -0.8674    0.0321    0.4964   47.3483
%     0.4968    0.0025    0.8678 -336.2277
%          0         0         0    1.0000