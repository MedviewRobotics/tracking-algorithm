function [x_origin,y_origin, z_origin] = findOrigin(mov,nFramesLeft,threshold,hblob,pivotOffset,stereoParams)

surgicalTip_3D = zeros(3,nFramesLeft);
surgicalTip_3D_norm = zeros(3,nFramesLeft);
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
        surgicalTip_3D(:, k) = surgicalTip_3D(:, k-1);
        surgicalTip_3D_norm(:, k) = surgicalTip_3D_norm(:, k-1);
    else
        [point3d_1,point3d_2, point3d_3] = findWorldCoordinates(centroidLeft,centroidRight,stereoParams);
        [surgicalTip_3D(:, k), rotMatrix] = findSurgicalTip(point3d_1,point3d_2,point3d_3,pivotOffset);
        if k > 10
            [surgicalTip_3D_norm(:,k)] = weightedAverage(surgicalTip_3D(:,:), k);
        end
    end
end

x_origin = mean(surgicalTip_3D_norm(1, 15:nFramesLeft));
temp = -mean(max(surgicalTip_3D_norm(2, 15:220))); %,max(surgicalTip_3D_norm(2, 50:100))));
y_origin = temp - 100;
z_origin = mean(surgicalTip_3D_norm(3, 15:nFramesLeft)) - 75; %75 is the center depth/works with robot/for back and forth


end

