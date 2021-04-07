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
initialEstimateError = [1 1 1]*1e5;
MotionNoise = [25, 10, 10];
measurementNoise = 1000;
surgicalTip_3D = zeros(3, nFramesLeft);
point3d_1 = zeros(3, nFramesLeft);
point3d_2 = zeros(3, nFramesLeft);
point3d_3 = zeros(3, nFramesLeft);
trackedLocation_1 = zeros(3, nFramesLeft);
trackedLocation_2 = zeros(3, nFramesLeft);
trackedLocation_3 = zeros(3, nFramesLeft);

%% Tracking Video

isTrackInitialized = 0;
trackedLocation_1 = zeros(nFramesLeft,2);
trackedLocation_2 = zeros(nFramesLeft,2);
trackedLocation_3 = zeros(nFramesLeft,2); 
v = VideoWriter('kalman.avi');
v.FrameRate = 10;
open(v)
frames_skip = 1;
isTrackInitialized = 0;
for k = 1:frames_skip:nFramesLeft

%Read Frames
frameLeft = mov(k).readerLeft;
frameRight = mov(k).readerRight;

%Initate preprocessing of frames
[frameLeftGray,frameRightGray] = preprocessFrames(frameLeft,frameRight);

%Find centroids in left and right frames
[centroidLeft, bboxLeft, centroidRight, bboxRight] = ...
    findCentroids(frameLeftGray,frameRightGray,threshold,hblob);

%isObjectDetected = size(centroidLeft, 1) > 2;
%When the ball is first detected, the example creates a Kalman filter.

    %Validate position of centroids
    if size(centroidLeft) ~= [3 3] | size(centroidRight) ~= [3 3] | k == 125
%        warning(['Could not find marker(s) in frame: ', num2str(k)])
%         trackedLocation_1(:,k) = trackedLocation_1(:,k-1);
%         trackedLocation_2(:,k) = trackedLocation_2(:,k-1);
%         trackedLocation_3(:,k) = trackedLocation_3(:,k-1);
%        [surgicalTip_3D(:, k), rotMatrix] = findSurgicalTip(trackedLocation_1(:,k),trackedLocation_2(:,k),trackedLocation_3(:,k),pivotOffset);
        trackedLocation_1(k,:) = predict(kalmanFilter_1);
        trackedLocation_2(k,:) = predict(kalmanFilter_2);
        trackedLocation_3(k,:) = predict(kalmanFilter_3);
        label = 'Predicted';
    else
        %[point3d_1(:,k),point3d_2(:,k), point3d_3(:,k)] = findWorldCoordinates(centroidLeft,centroidRight,stereoParams);
        if isTrackInitialized == 0
            kalmanFilter_1 = configureKalmanFilter('ConstantAcceleration',...
                centroidLeft(1,:), initialEstimateError, MotionNoise,measurementNoise);
            kalmanFilter_2 = configureKalmanFilter('ConstantAcceleration',...
                centroidLeft(2,:), initialEstimateError, MotionNoise,measurementNoise);
            kalmanFilter_3 = configureKalmanFilter('ConstantAcceleration',...
                centroidLeft(3,:), initialEstimateError, MotionNoise,measurementNoise);
            isTrackInitialized = 1;
            %[surgicalTip_3D(:, k), rotMatrix] = findSurgicalTip(point3d_1(:,k),point3d_2(:,k),point3d_3(:,k),pivotOffset);
            label = '';
            circle_1 = zeros(0,3);
            circle_2 = zeros(0,3);
            circle_3 = zeros(0,3);
        else
            trackedLocation_1(k,:) = correct(kalmanFilter_1,centroidLeft(1,:));
            trackedLocation_2(k,:) = correct(kalmanFilter_2, centroidLeft(2,:));
            trackedLocation_3(k,:) = correct(kalmanFilter_3, centroidLeft(3,:));
            %[surgicalTip_3D(:, k), rotMatrix] = findSurgicalTip(trackedLocation_1(:,k),trackedLocation_2(:,k),trackedLocation_3(:,k),pivotOffset);
            label = 'Correct';
        end
        
    end
    
circle_1 = [trackedLocation_1(k,:) 5];
circle_2 = [trackedLocation_2(k,:) 5];
circle_3 = [trackedLocation_3(k,:) 5];
    
%predicted location
% pos_1(k,:) = trackedLocation_1;
% pos_2(k,:) = trackedLocation_2;
% pos_3(k,:) = trackedLocation_3;
labs{k} = label;

colorImage = insertObjectAnnotation(frameLeft,'circle',...
    circle_1,label,'Color','red');
colorImage = insertObjectAnnotation(colorImage,'circle',...
    circle_2,label,'Color','green');
colorImage = insertObjectAnnotation(colorImage,'circle',...
    circle_3,label,'Color','blue');
% colorImage = insertShape(frameLeft,'rectangle',bboxLeft(1,:),'Color','black',...
%         'LineWidth',3);

player(colorImage)
pause(0.1);
writeVideo(v,colorImage)


end
release(player);
close(v); 

%%
%Tracked coordinates over image 
figure;
imshow(frameLeft)
hold on
plot(trackedLocation_1(2:end,1),trackedLocation_1(2:end,2),'go-','LineWidth',1)
plot(trackedLocation_2(2:end,1),trackedLocation_2(2:end,2),'ro-','LineWidth',1)
plot(trackedLocation_3(2:end,1),trackedLocation_3(2:end,2),'bo-','LineWidth',1)
hold off
title('Movement of markers throughout all frames using Kalman')


figure;
hold on
plot(2:nFramesLeft,trackedLocation_1(2:end,1))
plot(2:nFramesLeft,trackedLocation_2(2:end,1))
plot(2:nFramesLeft,trackedLocation_3(2:end,1))
hold off
legend('Green Marker','Red Marker','Blue Marker')
title('Movement of markers in the x direction in pixel coordinates')




