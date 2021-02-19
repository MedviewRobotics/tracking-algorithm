function [stereoParams, estimationErrors] = stereoCalibrate()
    
pathLeft = 'C:\Users\azizu\Desktop\Ryerson\4th year\Fall Term\capstone\capstone\Code\Tracking_algorithm\Phase 1\Phase I Demo\Left\left_';
pathRight = 'C:\Users\azizu\Desktop\Ryerson\4th year\Fall Term\capstone\capstone\Code\Tracking_algorithm\Phase 1\Phase I Demo\Right\right_';

for i = 1:20
    imageFileNames1{i} = strcat(pathLeft, num2str(i), '.jpg');
    imageFileNames2{i} = strcat(pathRight, num2str(i), '.jpg');
end

% Detect checkerboards in images (define corners)
[imagePoints, boardSize, imagesUsed] = detectCheckerboardPoints(imageFileNames1, imageFileNames2);

% Generate world coordinates of the checkerboard keypoints
squareSize = 22;  % in units of 'millimeters' -- checkerboard provided by MedView allows for consistency in size
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

% Read one of the images from the first stereo pair -- used to define size of images
I1 = imread(imageFileNames1{1});
[mrows, ncols, ~] = size(I1);

% Calibrate the camera
[stereoParams, pairsUsed, estimationErrors] = estimateCameraParameters(imagePoints, worldPoints, ...
    'EstimateSkew', true, 'EstimateTangentialDistortion', true, ...
    'NumRadialDistortionCoefficients', 3, 'WorldUnits', 'millimeters', ...
    'InitialIntrinsicMatrix', [], 'InitialRadialDistortion', [], ...
    'ImageSize', [mrows, ncols]);


end

