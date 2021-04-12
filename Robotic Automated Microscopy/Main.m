%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% BME70B: Biomedical Engineering Capstone Design
% AT03 Robotic Automated Microscopy Marker Tracking and Robot Movement
%
% Author #1: Ginette Hartell - 500755250
% Author #2: Mohammad Aziz Uddin - 500754765
% Author #3: Jay Tailor - 500750496
% Author #4: Claudia Alonzo - 500745327
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialize Peter Corke's Toolbox
petercorkeinitialize();

%% Initialize Tracking System Parameters
tic; %Start initialize timer

% addpath(genpath('March 31 Trials'));
% load("stereoParamsMar31.mat");
addpath(genpath('Accuracy Trials'));
load("stereoParamsAccuracy.mat");
%addpath(genpath('Trial 18-19'));
%load("stereoParams18.mat");

% readerLeft = VideoReader('myLeftTrialNormalMovement.avi');
% readerRight = VideoReader('myRightTrialNormalMovement.avi');

%readerLeft = VideoReader('myLeftTrialNormal8.avi');
%readerRight = VideoReader('myRightTrialNormal8.avi');

% readerLeft = VideoReader('myLeftTrialHoriz5cm.avi');
% readerRight = VideoReader('myRightTrialHoriz5cm.avi');

% readerLeft = VideoReader('myLeftTrialHoriz10cm.avi');
% readerRight = VideoReader('myRightTrialHoriz10cm.avi');

readerLeft = VideoReader('myLeftTrialVert5cm.avi');
readerRight = VideoReader('myRightTrialVert5cm.avi');

%Set up for skipping n frames
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

%Initialize Arrays
threshold = 245; % Threshold for Grayscale
surgicalTip_3D = zeros(3, nFramesLeft);
Robot_Accuracy = zeros(3, nFramesLeft);
angle_test = zeros(3, nFramesLeft);
Q = zeros(10*nFramesLeft, 6);
kalmanFilter_1 = [];
kalmanFilter_2 = [];
kalmanFilter_3 = [];
point3d_1 = zeros(3, nFramesLeft);
point3d_2 = zeros(3, nFramesLeft);
point3d_3 = zeros(3, nFramesLeft);
trackedLocation_1 = zeros(3, nFramesLeft);
trackedLocation_2 = zeros(3, nFramesLeft);
trackedLocation_3 = zeros(3, nFramesLeft);
eul = zeros(3, nFramesLeft);
elapsed_1 = zeros(1, nFramesLeft);
elapsed_2 = zeros(1, nFramesLeft);
elapsed_3 = zeros(1, nFramesLeft);
elapsed_4 = zeros(1, nFramesLeft);

%Initialize variables
pivotOffset = 200; % 20cm offset from midpoint btwn blue and green
frames_skip = 1;
initialEstimateError = [1 1]*1e5;
MotionNoise = [25, 10];
% initialEstimateError = [1 1 1]*1e5;
% MotionNoise = [25, 10, 10];
measurementNoise = 10;

model = createpde;
h = importGeometry(model,'brain.stl');
rotate(h, -90,[0 0 0],[1 0 0]); %x-axis rotation
rotate(h, -90,[0 0 0],[0 0 1]); %z-axis rotation
translate(h, [-1233, 1521, 820]);
pdegplot(h)

%rot= zeros(3,nFramesLeft);
[x_origin,y_origin,z_origin,eul_init] = findOrigin(mov,nFramesLeft,threshold,hblob,pivotOffset,stereoParams);
[Robot,q0] = initializeMicroscope(x_origin,y_origin,z_origin,eul_init);


disp('Initialization Completed.');

elapsed_initialized = toc; %Assign toc to initialization time

%% Marker tracking and robot movement
isTrackInitialized = 0;
j = 0;

%Initialize Video Player
player = vision.DeployableVideoPlayer('Location',[10,100]);
v = VideoWriter('accuracy.avi');
v.FrameRate = 30;
open(v)

nFramesLeft = 235; %only for Demo4
for k = 1:frames_skip:nFramesLeft
    tic %Starts pre-processing timer

    %Read Frames
    frameLeft = mov(k).readerLeft;
    frameRight = mov(k).readerRight;

    %Initate preprocessing of frames
    [frameLeftGray,frameRightGray] = preprocessFrames(frameLeft,frameRight);

    %End preprocessing phase
    elapsed_1(k) = toc;

    %Start timer for finding tip
    tic;

    %Find centroids in left and right frames
    [centroidLeft, bboxLeft, centroidRight, bboxRight] = ...
        findCentroids(frameLeftGray,frameRightGray,threshold,hblob);

    %Validate position of centroids
    if size(centroidLeft) ~= [3 3] | size(centroidRight) ~= [3 3]
        warning(['Could not detect marker(s) in frame: ', num2str(k), ', using predicted locations'])
        if k == 1
            pause(1);
        else
            point3d_1(:,k) = point3d_1(:,k-1);
            point3d_2(:,k) = point3d_2(:,k-1);
            point3d_3(:,k) = point3d_3(:,k-1);
            trackedLocation_1(:,k) = trackedLocation_1(:,k-1);
            trackedLocation_2(:,k) = trackedLocation_2(:,k-1);
            trackedLocation_3(:,k) = trackedLocation_3(:,k-1);
            [surgicalTip_3D(:, k)] = surgicalTip_3D(:, k-1);
            eul(:,k) = eul(:, k-1);
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
            [surgicalTip_3D(:, k), eul(:,k), normal] = findSurgicalTip(point3d_1(:,k),point3d_2(:,k),point3d_3(:,k),pivotOffset);
        else
            trackedLocation_1(:,k) = correct(kalmanFilter_1, point3d_1(:,k));
            trackedLocation_2(:,k) = correct(kalmanFilter_2, point3d_2(:,k));
            trackedLocation_3(:,k) = correct(kalmanFilter_3, point3d_3(:,k));
            [surgicalTip_3D(:, k), eul(:,k), normal] = findSurgicalTip(trackedLocation_1(:,k),trackedLocation_2(:,k),trackedLocation_3(:,k),pivotOffset);
        end
        elapsed_2(k) = toc; %End find tip timer

        %Plotting in video player
       rgb = plotVideo(frameLeft,bboxLeft,centroidLeft,...
                k,point3d_1,point3d_2,point3d_3);
            
        player(rgb);
        writeVideo(v,rgb);
    end

    if rem(k, 5) == 0
        %Start control system timer
        tic;

        %Find location of microscope
        locationMicroscope = [surgicalTip_3D(1,k) - abs(50*normal(2)),surgicalTip_3D(2,k)-20,surgicalTip_3D(3,k)-abs(150*normal(1))];
        %RECALL: X is Y in PLot, Z is X in PLot, Y is neg(Z) in Plot
        [xMicroscope, yMicroscope, zMicroscope] = world2Microscope_Accuracy(locationMicroscope(1), locationMicroscope(2), locationMicroscope(3), x_origin, y_origin, z_origin); %World to Microscope Coordinate Mapping

        %Find location of tip in microscope coordinates
        [xTip, yTip, zTip] = world2Microscope_Accuracy(surgicalTip_3D(1, k), surgicalTip_3D(2, k), surgicalTip_3D(3, k), x_origin, y_origin, z_origin); %World to Microscope Coordinate Mapping
    
        %Initiate control system
        [q0,X,Y,Z,R,P,Ya,Q(k*10 - 9:k*10, :)] = moveMicroscope(xMicroscope, yMicroscope, zMicroscope, q0, Robot,eul(:,k));
        elapsed_3(k) = toc;
        %Plotting
        model = createpde;
        g = importGeometry(model,'Instrument_Plotting_v9.stl');
        rotate(g, 90,[0 0 0],[0 0 1]); %z-axis rotation to bring to 0 0 0
        rotate(g, rad2deg(eul(3,k)),[0 0 0],[1 0 0]); %x-axis rotation
        rotate(g, rad2deg(eul(1,k)),[0 0 0],[0 1 0]); %y-axis rotation
        rotate(g, rad2deg(eul(2,k)),[0 0 0],[0 0 1]); %z-axis rotation
        translate(g, [xTip, yTip, zTip]);
        pdegplot(h)
        hold on
        pdegplot(g)
        Robot.plot3d(Q(k*10 - 9:k*10, :),'view','y','path','C:\Users\ginet\OneDrive\Documents\MATLAB\Capstone\tracking-algorithm\Robotic Automated Microscopy\robot\data\ARTE4','floorlevel',-175,'base');
        %Robot.plot(Q(k*10 - 9:k*10, :));
        hold off
        %Log control system accuracy
        Robot_Accuracy(:,k) = [X,Y,Z];
        angle_test(:,k) = [R,P,Ya];
    end

end

release(player)
close(v);

%% Joint stress testing

%     Joint1(count*10 - 9:10*count,1) = Q(:,1);
%     Joint2(count*10 - 9:10*count,1) = Q(:,2);
%     Joint3(count*10 - 9:10*count,1) = Q(:,3);
%     Joint4(count*10 - 9:10*count,1) = Q(:,4);
%     Joint5(count*10 - 9:10*count,1) = Q(:,5);
%     Joint6(count*10 - 9:10*count,1) = Q(:,6);

%% Output performance metrics

[T, Equiv_FPS_Rate] = systemPerformance(elapsed_1,elapsed_2, elapsed_3);
disp(T);
fprintf('Equivalent FPS Rate: %3.2f \n', Equiv_FPS_Rate);

%% Output Accuracy Metrics
%Vert 50
TAcc = trackingAccuracy(surgicalTip_3D(2,:),50,Robot_Accuracy(3,:))
disp(TAcc);

%% Accuracy plots
lower = 7;
upper = 235;

figure;
subplot(321)
plot(surgicalTip_3D(1,lower:upper));
title('Surgical Tip Position X');
subplot(322)
plot(Robot_Accuracy(2,lower:upper));
title('Robot Effector Tip Position Y (Tracking X)');
subplot(323)
plot(surgicalTip_3D(2,lower:upper));
title('Surgical Tip Position Y');
subplot(324)
plot(Robot_Accuracy(3,lower:upper));
title('Robot Effector Tip Position Z (Tracking Y)');
subplot(325)
plot(surgicalTip_3D(3,lower:upper));
title('Surgical Tip Position Z');
subplot(326)
plot(Robot_Accuracy(1,lower:upper));
title('Normalized Surgical Tip Position X (Tracking Z)');

figure
subplot(321)
plot(point3d_1(1, lower:upper))
title('Raw Green Marker X')
subplot(322)
plot(trackedLocation_1(1, lower:upper))
title('Tracked Green Marker X')
subplot(323)
plot(point3d_1(2, lower:upper))
title('Raw Green Marker Y')
subplot(324)
plot(trackedLocation_1(2, lower:upper))
title('Tracked Green Marker Y')
subplot(325)
plot(point3d_1(3, lower:upper))
title('Raw Green Marker Z')
subplot(326)
plot(trackedLocation_1(3, lower:upper))
title('Tracked Green Marker Z')

figure
subplot(321)
plot(point3d_2(1, lower:upper))
title('Raw Blue Marker X')
subplot(322)
plot(trackedLocation_2(1, lower:upper))
title('Tracked Blue Marker X')
subplot(323)
plot(point3d_2(2, lower:upper))
title('Raw Blue Marker Y')
subplot(324)
plot(trackedLocation_2(2, lower:upper))
title('Tracked Blue Marker Y')
subplot(325)
plot(point3d_2(3, lower:upper))
title('Raw Blue Marker Z')
subplot(326)
plot(trackedLocation_2(3, lower:upper))
title('Tracked Blue Marker Z')

figure
subplot(321)
plot(point3d_3(1, lower:upper))
title('Raw Red Marker X')
subplot(322)
plot(trackedLocation_3(1, lower:upper))
title('Tracked Red Marker X')
subplot(323)
plot(point3d_3(2, 52:150))
title('Raw Red Marker Y')
subplot(324)
plot(trackedLocation_3(2, lower:upper))
title('Tracked Red Marker Y')
subplot(325)
plot(point3d_3(3, 52:150))
title('Raw Red Marker Z')
subplot(326)
plot(trackedLocation_3(3, lower:upper))
title('Tracked Red Marker Z')

%% Ginette orientation mess-around

locationMicroscope = surgicalTip_3D(:,k) + normal(:)*60;

figure
plot3(point3d_1(1,k), point3d_1(2,k), point3d_1(3,k), '.g');
hold on
plot3(point3d_2(1,k), point3d_2(2,k), point3d_2(3,k), '.r');
plot3(point3d_3(1,k), point3d_3(2,k), point3d_3(3,k), '.b');
plot3(surgicalTip_3D(1,k), surgicalTip_3D(2,k), surgicalTip_3D(3,k), '.m');
plot3(locationMicroscope(1), locationMicroscope(2), locationMicroscope(3), '.c');
hold off