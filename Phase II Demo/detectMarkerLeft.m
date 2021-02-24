function [centroidRedLeft,bboxRedLeft,centroidGreenLeft,bboxGreenLeft,centroidBlueLeft,bboxBlueLeft] = detectMarkerLeft(frameLeft)

redThresh = 0.05; % Threshold for red detection
greenThresh = 0.05; % Threshold for green detection
blueThresh = 0.244; % Threshold for blue detection
%radii for disk structural element in Image dilation (determined
%experimentally)
radius_red = 8;
radius_green = 10;
radius_blue = 10;

hblob = vision.BlobAnalysis('AreaOutputPort', false, ... % Set blob analysis handling
                                'CentroidOutputPort', true, ... 
                                'BoundingBoxOutputPort', true', ...
                                'MinimumBlobArea', 1, ...
                                'MaximumBlobArea', 20000, ...
                                'MaximumCount',3);


binFrameRedLeft = detectmarkerColor(frameLeft,redThresh,1,radius_red);
binFrameGreenLeft = detectmarkerColor(frameLeft,greenThresh,2,radius_green);
binFrameBlueLeft = detectmarkerColor(frameLeft,blueThresh,3,radius_blue);

%Blob Analysis of each color
%LEFT
[centroidRedLeft,bboxRedLeft] = step(hblob,binFrameRedLeft);
[centroidGreenLeft,bboxGreenLeft] = step(hblob,binFrameGreenLeft);
[centroidBlueLeft,bboxBlueLeft] = step(hblob,binFrameBlueLeft);




end

