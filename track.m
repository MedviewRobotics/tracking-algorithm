%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% BME70B: Biomedical Engineering Capstone Design
% AT03 Robotic Automated Microscopy Calibration and Tracking
%
% Author #1: Mohammad Aziz Uddin - 500754765 (Tracking algorithm)
% Author #2: Ginette Hartell - 500755250 (Calibration)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Add relevant paths and load stereoParams if already calibrated
addpath(genpath('Trials 10-11'));
load('stereoParams10.mat')

%% Import videos to workspace
readerLeft= VideoReader('myVideoLeftTrial10.avi');
readerRight= VideoReader('myVideoRightTrial10.avi');

%% Calculate camera parameters using calibration image pairs 
%Call stereoCalibrate Function
% [stereoParams, estimationErrors] = stereoCalibrate();

%% Read and Rectify Video Frames
% left and right video must be rectified to compute disparity and
% reconstruct the 3-D scene

frameLeft = readFrame(readerLeft);
frameRight = readFrame(readerRight);

imshowpair(frameLeft,frameRight)

[frameLeftRect,frameRightRect] = rectifyStereoImages(frameLeft,frameRight,...
    stereoParams);

figure;
imshow(stereoAnaglyph(frameLeftRect,frameRightRect))
title('Rectified Video Frames')

%% Compute Disparity
% match ecorresponding points in stereo images on same pixel row.
% For each pixel in the left image compute the distance to the corresponding 
% right image

frameLeftGray = rgb2gray(frameLeftRect);
frameRightGray = rgb2gray(frameRightRect);

% 
frameLeftGray_eq = histeq(frameLeftGray);
figure;imshowpair(frameLeftGray,frameLeftGray_eq,'montage')

figure;subplot(121);imhist(frameLeftGray);subplot(122);imhist(frameLeftGray_eq);

disparityMap = disparitySGM(frameLeftGray,frameRightGray);
figure
imshow(disparityMap,[0 64]);
title('Disparity Map')
colormap jet
colorbar

%% 3-D Reconstruct scene

points3D = reconstructScene(disparityMap,stereoParams);

% Convert to meters and create a pointCloud object
points3D = points3D ./ 1000; % 
ptCloud = pointCloud(points3D,'Color',frameLeftRect);

% Create a streaming point cloud viewer
player3D = pcplayer([-3,3],[-3,3],[0,8],'VerticalAxis','y',...
    'VerticalAxisDir','Down');

% Visualize the point cloud
view(player3D,ptCloud);


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


%% Distance of each person to camera

% Find 3d world coordinates of the centroid of each detected person
% and compute distance from cetroid to camera in meters

centroids = [round(bboxes(:,1)+ bboxes(:,3)/2),...
    round(bboxes(:,2)+bboxes(:,2)/2)];

% Find the 3-D world coordinates of the centroids 
centroidsIdx = sub2ind(size(disparityMap),centroids(:,2),centroids(:,1));
X = points3D(:,:,1);
Y = points3D(:,:,2);
Z = points3D(:,:,3);
centroids3D = [X(centroidsIdx)';Y(centroidsIdx)';Z(centroidsIdx)'];

% Find distance from camera in meters 
dists = sqrt(sum(centroids3D.^2));

% Display the detected people and their distances
labels = cell(1,numel(dists));
for i = 1:numel(dists)
    labels{i} = sprintf('%0.2f meters',dists(i));
end

figure;
imshow(insertObjectAnnotation(frameLeftRect,'rectangle',bboxes,labels));
title('Detected people')

%% Process the rest of the video

while hasFrame(readerLeft) && hasFrame(readerRight)
% read the frames
frameLeft = readFrame(readerLeft);
frameRight = readFrame(readerRight);

%Rectify the frames
[frameLeftRect,frameRightRect] = ...
    rectifyStereoImages(frameLeft,frameRight,stereoParams);

%convert to greyscale
frameLeftGray = rgb2gray(frameLeftRect);
frameRightGray = rgb2gray(frameRightRect);

%Compute to disparity
%disparityMap = disparitySGM(frameLeftGray,frameRightGray);
disparityMap = disparityBM(frameLeftGray,frameRightGray);

% Reconstruct 3-D scene
points3D = reconstructScene(disparityMap,stereoParams);
points3D = points3D./1000;
%ptCloud = pointCloud(points3D,'Color',frameLeftRect);

% Create a streaming point cloud viewer
%player3D = pcplayer([-3,3],[-3,3],[0,8],'VerticalAxis','y',...
%    'VerticalAxisDir','Down');

% Visualize the point cloud
%view(player3D,ptCloud);

%table(points3D(1:514))

%detect people
bboxes = peopleDetector.step(frameLeftGray);

if ~isempty(bboxes)
    %find the centroids of detected people
    % Find the centroids of detected people.
        centroids = [round(bboxes(:, 1) + bboxes(:, 3) / 2), ...
            round(bboxes(:, 2) + bboxes(:, 4) / 2)];
        
        % Find the 3-D world coordinates of the centroids.
        centroidsIdx = sub2ind(size(disparityMap), centroids(:, 2), centroids(:, 1));
        X = points3D(:, :, 1);
        Y = points3D(:, :, 2);
        Z = points3D(:, :, 3);
        centroids3D = [X(centroidsIdx), Y(centroidsIdx), Z(centroidsIdx)];
        
        % Find the distances from the camera in meters.
        dists = sqrt(sum(centroids3D .^ 2, 2));
        
        % Display the detect people and their distances.
        labels = cell(1, numel(dists));
        for i = 1:numel(dists)
            labels{i} = sprintf('%0.2f meters', dists(i));
        end
        dispFrame = insertObjectAnnotation(frameLeftRect, 'rectangle', bboxes,...
            labels);
    else
        dispFrame = frameLeftRect;
    end
 
    % Display the frame.
    step(player, dispFrame);
end
    





















