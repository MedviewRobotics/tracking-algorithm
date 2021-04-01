%% Kalman filter testing grounds 

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


kalmanFilter = []; isTrackInitialized = false;
idx = 1;
initialEstimateError = [1 1 1]*1e5;
MotionNoise = [25, 10, 10];
measurementNoise = 25;
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




