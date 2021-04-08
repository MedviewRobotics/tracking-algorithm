function [x_origin, y_origin, z_origin, eul] = findOrigin(mov,nFramesLeft,threshold,hblob,pivotOffset,stereoParams)

surgicalTip_3D = zeros(3,nFramesLeft);
locationMicroscope = zeros(3,nFramesLeft);
eul = zeros(3,nFramesLeft);

kalmanFilter_1 = [];
kalmanFilter_2 = [];
kalmanFilter_3 = [];
point3d_1 = zeros(3, nFramesLeft);
point3d_2 = zeros(3, nFramesLeft);
point3d_3 = zeros(3, nFramesLeft);
trackedLocation_1 = zeros(3, nFramesLeft);
trackedLocation_2 = zeros(3, nFramesLeft);
trackedLocation_3 = zeros(3, nFramesLeft);
isTrackInitialized = 0;
initialEstimateError = [1 1]*1e5;
MotionNoise = [25, 10];
measurementNoise = 10;
temp = 0;

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
    
    %Validate position of centroids
    if size(centroidLeft) ~= [3 3] | size(centroidRight) ~= [3 3]
        if k ==1
            pause(1);
        else
            surgicalTip_3D(:, k) = surgicalTip_3D(:, k-1);
        end
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
            [surgicalTip_3D(:, k), eul(:,k), normal] = findSurgicalTip(point3d_1(:,k),point3d_2(:,k),point3d_3(:,k),pivotOffset);
            locationMicroscope(:,k) = surgicalTip_3D(:,k) + normal(:)*60;
        else
            trackedLocation_1(:,k) = correct(kalmanFilter_1, point3d_1(:,k));
            trackedLocation_2(:,k) = correct(kalmanFilter_2, point3d_2(:,k));
            trackedLocation_3(:,k) = correct(kalmanFilter_3, point3d_3(:,k));
            [surgicalTip_3D(:, k), eul(:,k), normal] = findSurgicalTip(trackedLocation_1(:,k),trackedLocation_2(:,k),trackedLocation_3(:,k),pivotOffset);
            locationMicroscope(:,k) = surgicalTip_3D(:,k) - normal(:)*60;
        end
    end
end

x_origin = mean(locationMicroscope(1, 15:nFramesLeft));
temp = -mean(max(locationMicroscope(2, 15:220))); %,max(surgicalTip_3D_norm(2, 50:100))));
y_origin = temp - 100;
z_origin = mean(locationMicroscope(3, 15:nFramesLeft)); %75 is the center depth/works with robot/for back and forth
eul = eul(:,5);
end

