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

initialEstimateError = [1 1 1]*1e5;
MotionNoise = [25, 10, 10];
measurementNoise = 1000;

initialEstimateError = [0.5 0.5 0.5]*1e5;
MotionNoise = [5, 5, 5];
measurementNoise = 0.2;

surgicalTip_3D = zeros(3, nFramesLeft);
point3d_1 = zeros(3, nFramesLeft);
point3d_2 = zeros(3, nFramesLeft);
point3d_3 = zeros(3, nFramesLeft);
red_marker = zeros(2, nFramesLeft);
trackedLocation_1 = zeros(4, nFramesLeft);
trackedLocation_2 = zeros(4, nFramesLeft);
trackedLocation_3 = zeros(4, nFramesLeft);
pcorr_1 = zeros (4, 4, nFramesLeft); 
pcorr_2 = zeros(4, 4, nFramesLeft); 
pcorr_3 = zeros(4, 4, nFramesLeft); 



%% 2D Kalman

tracks = initializeTracks();
% function predictNewLocationsOfTracks()
% for i = 1:length(tracks)
%     bbox = tracks(i).bbox;
%     
%     % Predict the current location of the track.
%     predictedCentroid = predict(tracks(i).kalmanFilter);
%     
%     % Shift the bounding box so that its center is at
%     % the predicted location.
%     predictedCentroid = int32(predictedCentroid) - bbox(3:4) / 2;
%     tracks(i).bbox = [predictedCentroid, bbox(3:4)];
% end
% end
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
        predict(kalmanFilter)
        trackedLocation = correct(kalmanFilter, centroidLeft(1,:));
        label = 'Corrected';
    else
        trackedLocation = predict(kalmanFilter);
        label = 'Predicted';
        %cost #of detections and # of tracks
        costMatrix = distance(kalmanFilter,trackedLocation);
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

%% 3D Kalman Extended

trackedLocation_1 = zeros(4, nFramesLeft);
trackedLocation_2 = zeros(4, nFramesLeft);
trackedLocation_3 = zeros(4, nFramesLeft);
pcorr_1 = zeros (4, 4, nFramesLeft); 
pcorr_2 = zeros(4, 4, nFramesLeft); 
pcorr_3 = zeros(4, 4, nFramesLeft); 

kalmanFilter_1 = trackingUKF;
kalmanFilter_2 = trackingUKF;
kalmanFilter_3 = trackingUKF;

frames_skip = 1;
isTrackInitialized = 0;

for k = 1:frames_skip:235
    %Read Frames
    frameLeft = mov(k).readerLeft;
    frameRight = mov(k).readerRight;
    
    %Initate preprocessing of frames
    [frameLeftGray,frameRightGray] = preprocessFrames(frameLeft,frameRight);
    
    %Find centroids in left and right frames
    [centroidLeft, bboxLeft, centroidRight, bboxRight] = ...
        findCentroids(frameLeftGray,frameRightGray,threshold,hblob);
    
    %Validate position of centroids
    if size(centroidLeft) ~= [3 3] | size(centroidRight) ~= [3 3]
        warning(['Could not find marker(s) in frame: ', num2str(k)])
        point3d_1(:,k) = point3d_1(:,k-1);
        point3d_2(:,k) = point3d_2(:,k-1);
        point3d_3(:,k) = point3d_3(:,k-1);
        trackedLocation_1(:,k) = trackedLocation_1(:,k-1);
        trackedLocation_2(:,k) = trackedLocation_2(:,k-1);
        trackedLocation_3(:,k) = trackedLocation_3(:,k-1);

        [surgicalTip_3D(:, k), rotMatrix] = findSurgicalTip(trackedLocation_1(:,k),trackedLocation_2(:,k),trackedLocation_3(:,k),pivotOffset);
        trackedLocation_1_1(:, k) = predict(kalmanFilter_1);
        trackedLocation_2_1(:, k) = predict(kalmanFilter_2);
        trackedLocation_3_1(:, k) = predict(kalmanFilter_3);
        label = 'Predicted';

%         [surgicalTip_3D(:, k), eul(:,k)] = findSurgicalTip(trackedLocation_1(:,k),trackedLocation_2(:,k),trackedLocation_3(:,k),pivotOffset);

    else
        [point3d_1(:,k),point3d_2(:,k), point3d_3(:,k)] = findWorldCoordinates(centroidLeft,centroidRight,stereoParams);
        trackedLocation_1(:,k) = correct(kalmanFilter_1, point3d_1(:,k));
        trackedLocation_2(:,k) = correct(kalmanFilter_2, point3d_2(:,k));
        trackedLocation_3(:,k) = correct(kalmanFilter_3, point3d_3(:,k));
%         [surgicalTip_3D(:, k), eul(:,k)] = findSurgicalTip(trackedLocation_1(:,k),trackedLocation_2(:,k),trackedLocation_3(:,k),pivotOffset);
    end
end
%% Plots

figure
subplot(211)
plot(trackedLocation_1(1,50:235))
subplot(212)
plot(point3d_1(1,50:235))


%% Ginette 2D Kalman Demo

frames_skip = 1;
isTrackInitialized = 0;

for k = 60:frames_skip:135
    %Read Frames
    frameLeft = mov(k).readerLeft;
    frameRight = mov(k).readerRight;
    
    %Initate preprocessing of frames
    [frameLeftGray,frameRightGray] = preprocessFrames(frameLeft,frameRight);
    
    %Find centroids in left and right frames
    [centroidLeft, bboxLeft, centroidRight, bboxRight] = ...
        findCentroids(frameLeftGray,frameRightGray,threshold,hblob);
    
    %Validate position of centroids
    if size(centroidLeft) == [2 2]
        warning(['Could not find marker(s) in frame: ', num2str(k)])
        red_marker(:, k) = red_marker(:, k-1);
        trackedLocation_1(:,k) = trackedLocation_1(:,k-1);
    else
        red_marker(:, k) = centroidLeft(2, :);
        if isTrackInitialized == 0
            kalmanFilter_1 = configureKalmanFilter('ConstantAcceleration',...
                red_marker(:, k), initialEstimateError, MotionNoise, measurementNoise);
            isTrackInitialized = 1;

            [surgicalTip_3D(:, k), rotMatrix] = findSurgicalTip(point3d_1(:,k),point3d_2(:,k),point3d_3(:,k),pivotOffset);
            label = ''; 
            circle_1 = zeros(0,3);
            circle_2 = zeros(0,3);
            circle_3 = zeros(0,3);
        else
            trackedLocation_1(:,k) = correct(kalmanFilter_1, point3d_1(:,k));
            trackedLocation_2(:,k) = correct(kalmanFilter_2, point3d_2(:,k));
            trackedLocation_3(:,k) = correct(kalmanFilter_3, point3d_3(:,k));
            [surgicalTip_3D(:, k), rotMatrix] = findSurgicalTip(trackedLocation_1(:,k),trackedLocation_2(:,k),trackedLocation_3(:,k),pivotOffset);
            label = 'Correct';

        else
            trackedLocation_1(:,k) = correct(kalmanFilter_1, red_marker(:, k));

        end
        
        %Cost assignments for each marker based on detection and prediction
%         costMatrix_1 = distance(point3d_1(:,k),trackedLocation_1(:,k));
%         costMatrix_2 = distance(point3d_2(:,k),trackedLocation_2(:,k) );
%         costMatrix_3 = distance(point3d_3(:,k),trackedLocation_3(:,k));
        
        %Cost estimation found through predicted and detected values
        %kalmanFilter_X holds previous state, meas_noise,process_noise,etc
        pred_1 = predict(kalmanFilter_1);
        costMatrix_1 = distance(kalmanFilter_1,point3d_1(:,k));
        pred_2 = predict(kalmanFilter_2);
        costMatrix_2 = distance(kalmanFilter_2,point3d_2(:,k));
        pred_3 = predict(kalmanFilter_3);
        costMatrix_3 = distance(kalmanFilter_3,point3d_3(:,k));
        
        %put into one matrix to mimic all
        cost_test_1 = [costMatrix_1,costMatrix_2,costMatrix_3];
        
        %ignore kalman noise and see if assignment works with just raw
        %detection and predicted values
%         detection_test = [point3d_1(:,k),point3d_2(:,k),point3d_3(:,k)];
%         track_test = [pred_1;pred_2;pred_3];
        %cost_test = distance(detection_test,track_test);
        
        %Data associations. assign each marker a track, assignments_test
        %should ideally give 1 2 3, indicating each marker has its own
        %track however this provides unassignedDetection as 1,2,3 and not
        %sure why
        [assignments_test,unassignedTracks_test,unassignedDetections_test] = ...
            assignDetectionsToTracks(cost_test_1,0.2)
                
        
        %assign detections to track using cost matrix and cost of
        %non-assignment, this gives [1 1] for each as it makes sense since
        %only one marker is being entered 
        [assignments_1,unassignedTracks_1,unassignedDetections_1] = ...
            assignDetectionsToTracks(costMatrix_1,10);
        [assignments_2,unassignedTracks_2,unassignedDetections_2] = ...
            assignDetectionsToTracks(costMatrix_2,10);  
        [assignments_3,unassignedTracks_3,unassignedDetections_3] = ...
            assignDetectionsToTracks(costMatrix_3,10);  
        
    end
    
    labs{k} = label;
    circle_1 = trackedLocation_1(:,k);
    circle_2 = trackedLocation_2(:,k);
    circle_3 = trackedLocation_3(:,k);
    
    %Can't display 3D coordinates because 2D image
%     colorImage = insertObjectAnnotation(frameLeft,'circle',...
%         circle_1,label,'Color','red');
%     colorImage = insertObjectAnnotation(colorImage,'circle',...
%         circle_2',label,'Color','green');
%     colorImage = insertObjectAnnotation(colorImage,'circle',...
%         circle_3',label,'Color','blue');
%     
%     player(colorImage)
end
release(player)

%% Accuracy table and plots 

%% Plot frames from demo
textLabel_1 = sprintf('Detected');
textLabel_2 = sprintf('Corrected');

frame1 = 65;
frame2 = 75;
frame3 = 95;
frame4 = 110;
frame5 = 120;
frame6 = 129;

%Frame 15
figure
subplot(321)
imshow(mov(frame1).readerLeft)
hold on
plot(red_marker(1,frame1),red_marker(2,frame1),'b.')
text(red_marker(1,frame1), red_marker(2,frame1)+ 2, textLabel_1, 'Color', 'b');
plot(trackedLocation_1(1,frame1),trackedLocation_1(2,frame1),'m.')
text(trackedLocation_1(1,frame1), trackedLocation_1(2,frame1)- 2, textLabel_2, 'Color', 'm');
hold off
subplot(322)
imshow(mov(frame2).readerLeft)
hold on
plot(red_marker(1,frame2),red_marker(2,frame2),'b.')
text(red_marker(1,frame2), red_marker(2,frame2)+ 2, textLabel_1, 'Color', 'b');
plot(trackedLocation_1(1,frame2),trackedLocation_1(2,frame2),'m.')
text(trackedLocation_1(1,frame2), trackedLocation_1(2,frame2)- 2, textLabel_2, 'Color', 'm');
hold off
subplot(323)
imshow(mov(frame3).readerLeft)
hold on
plot(red_marker(1,frame3),red_marker(2,frame3),'b.')
text(red_marker(1,frame3), red_marker(2,frame3)+ 2, textLabel_1, 'Color', 'b');
plot(trackedLocation_1(1,frame3),trackedLocation_1(2,frame3),'m.')
text(trackedLocation_1(1,frame3), trackedLocation_1(2,frame3)- 2, textLabel_2, 'Color', 'm');
hold off
subplot(324)
imshow(mov(frame4).readerLeft)
hold on
plot(red_marker(1,frame4),red_marker(2,frame4),'b.')
text(red_marker(1,frame4), red_marker(2,frame4)+ 2, textLabel_1, 'Color', 'b');
plot(trackedLocation_1(1,frame4),trackedLocation_1(2,frame4),'m.')
text(trackedLocation_1(1,frame4), trackedLocation_1(2,frame4)- 2, textLabel_2, 'Color', 'm');
hold off
subplot(325)
imshow(mov(frame5).readerLeft)
hold on
plot(red_marker(1,frame5),red_marker(2,frame5),'b.')
text(red_marker(1,frame5), red_marker(2,frame5)+ 2, textLabel_1, 'Color', 'b');
plot(trackedLocation_1(1,frame5),trackedLocation_1(2,frame5),'m.')
text(trackedLocation_1(1,frame5), trackedLocation_1(2,frame5)- 2, textLabel_2, 'Color', 'm');
hold off
subplot(326)
imshow(mov(frame6).readerLeft)
hold on
plot(red_marker(1,frame6),red_marker(2,frame6),'b.')
text(red_marker(1,frame6), red_marker(2,frame6)+ 2, textLabel_1, 'Color', 'b');
plot(trackedLocation_1(1,frame6),trackedLocation_1(2,frame6),'m.')
text(trackedLocation_1(1,frame6), trackedLocation_1(2,frame6)- 2, textLabel_2, 'Color', 'm');
hold off

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

TAcc = trackingAccuracy(surgicalTip_3D(1,:),50,surgicalTip_3D(1,:))
disp(TAcc);

%%
predictions = [1,1;2,2];
detections = [1.1,1.1;2.1,2.1;1.5,3];
cost = zeros(size(track_test,1),size(detection_test,1));
for i = 1:size(track_test, 1)
      diff = detection_test - repmat(track_test(i,:),[size(detection_test,1),1]);
      cost(i, :) = sqrt(sum(diff .^ 2,2));
end
[assignment,unassignedTracks,unassignedDetections] = ...
            assignDetectionsToTracks(cost,0.2)
  figure;
  plot(track_test(:,1),track_test(:,2),'*',detection_test(:,1),...
            detection_test(:,2),'ro');
  hold on;
  legend('predictions','detections');
  for i = 1:size(assignment,1)
    text(track_test(assignment(i, 1),1)+0.1,...
            track_test(assignment(i,1),2)-0.1,num2str(i));
    text(detection_test(assignment(i, 2),1)+0.1,...
            detection_test(assignment(i,2),2)-0.1,num2str(i));
  end
  for i = 1:length(unassignedDetections)
    text(detection_test(unassignedDetections(i),1)+0.1,...
            detection_test(unassignedDetections(i),2)+0.1,'unassigned');
  end
  xlim([0,4]);
  ylim([0,4]);
 
  
  
% 
% TAcc = trackingAccuracy(surgicalTip_3D(1,52:150),50,surgicalTip_3D(1,52:150))
% disp(TAcc);

