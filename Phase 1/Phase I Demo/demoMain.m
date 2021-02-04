%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% BME70B: Biomedical Engineering Capstone Design
% AT03 Robotic Automated Microscopy Calibration and Tracking
% Phase I Demonstration
%
% Author #1: Ginette Hartell - 500755250
% Author #2: Mohammad Aziz Uddin - 500754765
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Local Directory
addpath(genpath('Phase 1'));

%% Play short clip of the left input video 
winopen('myVideoLeftTrial9.avi')

%% Play short clip of the right input video 
winopen('myVideoRightTrial9.avi')

%% Show example left image for calibration
winopen('left_13.jpg')

%% Show example right image for calibration
winopen('right_13.jpg')

%% Import videos to workspace
readerLeft= VideoReader('myVideoLeftTrial9.avi');
readerRight= VideoReader('myVideoRightTrial9.avi');

%% Calculate camera parameters using calibration image pairs 
%Call stereoCalibrate Function
[stereoParams, estimationErrors] = stereoCalibrate();

%% Demonstrate rectification
% Grab a frame pair to plot a demonstration
leftFrame = read(readerLeft, 175);
rightFrame = read(readerRight, 175);

%Rectify Images
[R1 R2] = rectifyStereoImages(leftFrame, rightFrame, stereoParams);

%Plot images before and after rectification in one subplot
figure
subplot(2, 2, 1)
imshow(leftFrame)
title('Left Frame #175 Before Rectification');
subplot(2, 2, 2)
imshow(rightFrame)
title('Right Frame #175 Before Rectification');
%figure
subplot(2, 2, 3)
imshow(R1)
title('Left Frame #175 After Rectification');
subplot(2, 2, 4)
imshow(R2)
title('Right Frame #175 After Rectification');

%Create the stereo anaglyph of the rectified stereo pair image and display it
A = stereoAnaglyph(R1,R2);
figure;
imshow(A);
title('Anaglyph Composite View of the Rectified Stereo Pair Image');
%% Tracker detection demonstration:

% To play tracking video
player = vision.DeployableVideoPlayer('Location',[10,100]);
v = VideoWriter('OTS_Demo1.mp4');
open(v)
% Thresholds determined experimentally and through colorThresholder App
redThresh = 0.208; % Threshold for red detection
greenThresh = 0.224; % Threshold for green detection
blueThresh = 0.244; % Threshold for blue detection

%Blob analysis --> Analyze binary image and determine size, shape,
%centroid, bouding box
hblob = vision.BlobAnalysis('AreaOutputPort', false, ... % Set blob analysis handling
                                'CentroidOutputPort', true, ... 
                                'BoundingBoxOutputPort', true', ...
                                'MinimumBlobArea', 1, ...
                                'MaximumBlobArea', 20000, ...
                                'MaximumCount',3);
                            
%radii for disk structural element in Image dilation (determined
%experimentally)
radius_red = 1;
radius_green = 10;
radius_blue = 1;                            

%Marker tracking 
while hasFrame(readerLeft)
    
    %Find markers
    image = readFrame(readerLeft);
    [binFrameRed,redCentroids] = detectmarkerColor(image,redThresh,1,radius_red);
    [binFrameGreen,greenCentroids] = detectmarkerColor(image,greenThresh,2,radius_green);
    [binFrameBlue,blueCentroids] = detectmarkerColor(image,blueThresh,3,radius_blue);
    
    %Blob Analysis of each color
    [centroidRed,bboxRed] = step(hblob,binFrameRed);
    [centroidGreen,bboxGreen] = step(hblob,binFrameGreen);
    [centroidBlue,bboxBlue] = step(hblob,binFrameBlue);

     %RED
     rgb = insertShape(image,'rectangle',bboxRed(1,:),'Color','red',...
         'LineWidth',3);
     %GREEN
     rgb = insertShape(rgb,'rectangle',bboxGreen(1,:),'Color','green',...
         'LineWidth',3);
     %BLUE
     rgb = insertShape(rgb,'rectangle',bboxBlue(1,:),'Color','blue',...
         'LineWidth',3);
     rgb = insertText(rgb,centroidRed(1,:) + 20,['X: ' num2str(round(centroidRed(1,1)),'%d')...
         ' Y: ' num2str(round(centroidRed(1,2)),'%d')],'FontSize',18);
     rgb = insertText(rgb,centroidGreen(1,:)+15,['X: ' num2str(round(centroidGreen(1,1)),'%d')...
         ' Y: ' num2str(round(centroidGreen(1,2)),'%d')],'FontSize',18);
     rgb = insertText(rgb,centroidBlue(1,:)-75,['X: ' num2str(round(centroidBlue(1,1)),'%d')...
         ' Y: ' num2str(round(centroidBlue(1,2)),'%d')],'FontSize',18);
     
     player(rgb);
     writeVideo(v,rgb);
end

release(player);
close(v);







