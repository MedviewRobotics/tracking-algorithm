function [trackedLocation_1,trackedLocation_2,trackedLocation_3,surgicalTip_3D,elapsed_2] = ...
    kalmanFilters(centroidLeft,centroidRight,k,nFramesLeft,pivotOffset,stereoParams,trackedLocation_1,...
trackedLocation_2,trackedLocation_3,surgicalTip_3D)


elapsed_2 = zeros(1, nFramesLeft);
eul = zeros(3, nFramesLeft);
initialEstimateError = [1 1]*1e5;
MotionNoise = [25, 10];
measurementNoise = 10;

if size(centroidLeft) ~= [3 3] | size(centroidRight) ~= [3 3]
%        warning(['Could not find marker(s) in frame: ', num2str(k)])
    if k == 1
        pause(1);
    else
        trackedLocation_1(:,k) = predict(kalmanFilter_1);
        trackedLocation_2(:,k) = predict(kalmanFilter_2);
        trackedLocation_3(:,k) = predict(kalmanFilter_3);
        [surgicalTip_3D(:, k), rotMatrix] = findSurgicalTip(trackedLocation_1(:,k),trackedLocation_2(:,k),trackedLocation_3(:,k),pivotOffset);
    end
        elapsed_2(k) = toc; %End find tip timer
    else
        [point3d_1(:,k),point3d_2(:,k), point3d_3(:,k)] = findWorldCoordinates(centroidLeft,centroidRight,stereoParams);
        if k == 1
            kalmanFilter_1 = configureKalmanFilter('ConstantVelocity',...
                point3d_1(:,k), initialEstimateError, MotionNoise,measurementNoise);
            kalmanFilter_2 = configureKalmanFilter('ConstantVelocity',...
                point3d_2(:,k), initialEstimateError, MotionNoise,measurementNoise);
            kalmanFilter_3 = configureKalmanFilter('ConstantVelocity',...
                point3d_3(:,k), initialEstimateError, MotionNoise,measurementNoise);
            [surgicalTip_3D(:, k), rotMatrix] = findSurgicalTip(point3d_1(:,k),point3d_2(:,k),point3d_3(:,k),pivotOffset);
        else
            trackedLocation_1(:,k) = correct(kalmanFilter_1,point3d_1(:,k));
            trackedLocation_2(:,k) = correct(kalmanFilter_2, point3d_2(:,k));
            trackedLocation_3(:,k) = correct(kalmanFilter_3, point3d_3(:,k));
            [surgicalTip_3D(:, k), eul(:,k), normal] = findSurgicalTip(trackedLocation_1(:,k),trackedLocation_2(:,k),trackedLocation_3(:,k),pivotOffset);
            label = 'Correct';
        end
        elapsed_2(k) = toc; %End find tip timer
end


end

