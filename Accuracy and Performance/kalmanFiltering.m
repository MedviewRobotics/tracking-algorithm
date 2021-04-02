%% Kalman filter testing grounds

addpath(genpath('Accuracy Trials'));
load("stereoParamsAccuracy.mat");

pivotOffset = 200; % 20cm offset from midpoint btwn blue and green
threshold = 245; % Threshold for Grayscale

readerLeft = VideoReader('myLeftTrialHoriz5cm.avi');
readerRight = VideoReader('myRightTrialHoriz5cm.avi');

player = vision.DeployableVideoPlayer('Location',[10,100]);

nFramesLeft = readerLeft.NumFrames;
vidHeightLeft = readerLeft.Height;
vidWidthLeft = readerLeft.Width;
nFramesRight = readerRight.NumFrames;
vidHeightRight = readerRight.Height;
vidWidthRight = readerRight.Width;

mov(1:nFramesLeft) = ...
    struct('readerLeft',zeros(vidHeightLeft,vidWidthLeft, 3,'uint8'),...
    'readerRight',zeros(vidHeightRight,vidWidthRight, 3,'uint8'),...
    'colormap',[]);

for k = 1:nFramesLeft
    mov(k).readerLeft = read(readerLeft,k);
    mov(k).readerRight = read(readerRight,k);
end

% Set blob analysis handling
hblob = vision.BlobAnalysis('AreaOutputPort', false, ...
    'CentroidOutputPort', true, ...
    'BoundingBoxOutputPort', true', ...
    'MinimumBlobArea', 1, ...
    'MaximumBlobArea', 20000, ...
    'MaximumCount',3);

kalmanFilter = [];
isTrackInitialized = false;
initialEstimateError = [1 1]*1e5;
MotionNoise = [25, 10];
measurementNoise = 10;
surgicalTip_3D = zeros(3, nFramesLeft);
point3d_1 = zeros(3, nFramesLeft);
point3d_2 = zeros(3, nFramesLeft);
point3d_3 = zeros(3, nFramesLeft);
trackedLocation_1 = zeros(3, nFramesLeft);
trackedLocation_2 = zeros(3, nFramesLeft);
trackedLocation_3 = zeros(3, nFramesLeft);

%% 2D Kalman

frames_skip = 1;


for k = 1:frames_skip:nFramesLeft

%Read Frames
frameLeft = mov(k).readerLeft;
frameRight = mov(k).readerRight;

%Initate preprocessing of frames
[frameLeftGray,frameRightGray] = preprocessFrames(frameLeft,frameRight);

%Find centroids in left and right frames
[centroidLeft, bboxLeft, centroidRight, bboxRight] = ...
    findCentroids(frameLeftGray,frameRightGray,threshold,hblob);

isObjectDetected = size(centroidLeft, 1) > 2;
%When the ball is first detected, the example creates a Kalman filter.
if ~isTrackInitialized
    if isObjectDetected
        kalmanFilter = configureKalmanFilter('ConstantAcceleration',...
            centroidLeft(1,:), initialEstimateError, MotionNoise,measurementNoise);
        isTrackInitialized = true;
    end
    label = ''; circle = zeros(0,3);
else
    if isObjectDetected
        predict(kalmanFilter);
        trackedLocation = correct(kalmanFilter, centroidLeft(1,:));
        label = 'Corrected';
    else
        trackedLocation = predict(kalmanFilter);
        label = 'Predicted';
        %cost #of detections and # of tracks
        costMatrix = distance(centroidLeft(1,:),trackedLocation);
        %costMatrix = diag(costMatrix);
        %arbitray cost of nonassignment = 10
        [assignments,unassignedTracks,unassignedDetections] = assignDetectionsToTracks(costMatrix,10);
    end
    circle = [trackedLocation 5];
end
%predicted location
pos(k,:) = trackedLocation;
labs{k} = label;
colorImage = insertObjectAnnotation(frameLeft,'circle',...
    circle,label,'Color','red');
% colorImage = insertShape(frameLeft,'rectangle',bboxLeft(1,:),'Color','black',...
%         'LineWidth',3);

player(colorImage)
pause(0.1);


end
release(player);

figure;
imshow(frameLeft)
hold on
plot(pos(:,1),pos(:,2),'ro-','LineWidth',1)
hold off

figure;
plot(1:nFramesLeft,pos(:,1))

%% 3D Kalman


frames_skip = 1;
isTrackInitialized = 0;

for k = 50:frames_skip:150
    %Read Frames
    frameLeft = mov(k).readerLeft;
    frameRight = mov(k).readerRight;
    
    %Initate preprocessing of frames
    [frameLeftGray,frameRightGray] = preprocessFrames(frameLeft,frameRight);
    
    %Find centroids in left and right frames
    [centroidLeft, bboxLeft, centroidRight, bboxRight] = ...
        findCentroids(frameLeftGray,frameRightGray,threshold,hblob);
    
    %Validate position of centroids
    if size(centroidLeft) ~= [3 3] | size(centroidRight) ~= [3 3] | k == 125
        warning(['Could not find marker(s) in frame: ', num2str(k)])
        point3d_1(:,k) = point3d_1(:,k-1);
        point3d_2(:,k) = point3d_2(:,k-1);
        point3d_3(:,k) = point3d_3(:,k-1);
        trackedLocation_1(:,k) = trackedLocation_1(:,k-1);
        trackedLocation_2(:,k) = trackedLocation_2(:,k-1);
        trackedLocation_3(:,k) = trackedLocation_3(:,k-1);
        [surgicalTip_3D(:, k), rotMatrix] = findSurgicalTip(trackedLocation_1(:,k),trackedLocation_2(:,k),trackedLocation_3(:,k),pivotOffset);
    else
        [point3d_1(:,k),point3d_2(:,k), point3d_3(:,k)] = findWorldCoordinates(centroidLeft,centroidRight,stereoParams);
        if isTrackInitialized == 0
            kalmanFilter_1 = configureKalmanFilter('ConstantVelocity',...
                point3d_1(:,k), initialEstimateError, MotionNoise,measurementNoise);
            kalmanFilter_2 = configureKalmanFilter('ConstantVelocity',...
                point3d_2(:,k), initialEstimateError, MotionNoise,measurementNoise);
            kalmanFilter_3 = configureKalmanFilter('ConstantVelocity',...
                point3d_3(:,k), initialEstimateError, MotionNoise,measurementNoise);
            isTrackInitialized = 1;
            [surgicalTip_3D(:, k), rotMatrix] = findSurgicalTip(point3d_1(:,k),point3d_2(:,k),point3d_3(:,k),pivotOffset);
        else
            trackedLocation_1(:,k) = correct(kalmanFilter_1, point3d_1(:,k));
            trackedLocation_2(:,k) = correct(kalmanFilter_2, point3d_2(:,k));
            trackedLocation_3(:,k) = correct(kalmanFilter_3, point3d_3(:,k));
            [surgicalTip_3D(:, k), rotMatrix] = findSurgicalTip(trackedLocation_1(:,k),trackedLocation_2(:,k),trackedLocation_3(:,k),pivotOffset);
        end
    end
end


%%

figure
subplot(321)
plot(point3d_1(1, 52:150))
title('Raw Green Marker X')
subplot(322)
plot(trackedLocation_1(1, 52:150))
title('Tracked Green Marker X')
subplot(323)
plot(point3d_1(2, 52:150))
title('Raw Green Marker Y')
subplot(324)
plot(trackedLocation_1(2, 52:150))
title('Tracked Green Marker Y')
subplot(325)
plot(point3d_1(3, 52:150))
title('Raw Green Marker Z')
subplot(326)
plot(trackedLocation_1(3, 52:150))
title('Tracked Green Marker Z')

figure
subplot(321)
plot(point3d_2(1, 52:150))
title('Raw Blue Marker X')
subplot(322)
plot(trackedLocation_2(1, 52:150))
title('Tracked Blue Marker X')
subplot(323)
plot(point3d_2(2, 52:150))
title('Raw Blue Marker Y')
subplot(324)
plot(trackedLocation_2(2, 52:150))
title('Tracked Blue Marker Y')
subplot(325)
plot(point3d_2(3, 52:150))
title('Raw Blue Marker Z')
subplot(326)
plot(trackedLocation_2(3, 52:150))
title('Tracked Blue Marker Z')

figure
subplot(321)
plot(point3d_3(1, 52:150))
title('Raw Red Marker X')
subplot(322)
plot(trackedLocation_3(1, 52:150))
title('Tracked Red Marker X')
subplot(323)
plot(point3d_3(2, 52:150))
title('Raw Red Marker Y')
subplot(324)
plot(trackedLocation_3(2, 52:150))
title('Tracked Red Marker Y')
subplot(325)
plot(point3d_3(3, 52:150))
title('Raw Red Marker Z')
subplot(326)
plot(trackedLocation_3(3, 52:150))
title('Tracked Red Marker Z')

figure;
subplot(311)
plot(surgicalTip_3D(1,52:150));
title('Surgical Tip Position X');
subplot(312)
plot(surgicalTip_3D(2,52:150));
title('Surgical Tip Position Y');
subplot(313)
plot(surgicalTip_3D(3,52:150));
title('Surgical Tip Position Z');
% 
% TAcc = trackingAccuracy(surgicalTip_3D(1,52:150),50,surgicalTip_3D(1,52:150))
% disp(TAcc);