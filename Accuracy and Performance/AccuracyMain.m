%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% BME70B: Biomedical Engineering Capstone Design
% AT03 Robotic Automated Microscopy Marker Tracking and Robot Movement
% Phase II Demonstration
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

addpath(genpath('Accuracy Trials'));
load("stereoParamsAccuracy.mat");
%addpath(genpath('Trial 18-19'));
%load("stereoParams18.mat");

pivotOffset = 200; % 20cm offset from midpoint btwn blue and green
threshold = 245; % Threshold for Grayscale 

% readerLeft = VideoReader('myLeftTrialVert5cm.avi');
% readerRight = VideoReader('myRightTrialVert5cm.avi');

% readerLeft = VideoReader('myLeftTrialVert5cm.avi');
% readerRight = VideoReader('myRightTrialVert5cm.avi');

readerLeft = VideoReader('myLeftTrialNormal8.avi');
readerRight = VideoReader('myRightTrialNormal8.avi');

%readerLeft = VideoReader('myLeftTrialHoriz5cm.avi');
%readerRight = VideoReader('myRightTrialHoriz5cm.avi');

% readerLeft = VideoReader('myLeftTrialHoriz10cm.avi');
% readerRight = VideoReader('myRightTrialHoriz10cm.avi');

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

[Robot,q0] = initializeMicroscope();

[x_origin,y_origin, z_origin] = findOrigin(mov,nFramesLeft,threshold,hblob,pivotOffset,stereoParams);

disp('Initialization Completed.');

elapsed_initialized = toc; %Assign toc to initialization time

%% Marker tracking and robot movement
%close all;

%Initialize Arrays
surgicalTip_3D = zeros(3, nFramesLeft);
surgicalTip_3D_norm = zeros(3, nFramesLeft);
deviation = zeros(3, nFramesLeft);
Robot_Accuracy = zeros(3, nFramesLeft);
elapsed_1 = zeros(1, nFramesLeft);
elapsed_2 = zeros(1, nFramesLeft);
elapsed_3 = zeros(1, nFramesLeft);
elapsed_4 = zeros(1, nFramesLeft);
count = 1;
%Q = zeros(10, 6, nFramesLeft);
Q = zeros(10*nFramesLeft, 6);

%Initialize Video Player
player = vision.DeployableVideoPlayer('Location',[10,100]);
v = VideoWriter('accuracy.avi');
v.FrameRate = 30;
open(v)

frames_skip = 1;

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
    warning(['Could not find marker(s) in frame: ', num2str(k)])
    surgicalTip_3D(:, k) = surgicalTip_3D(:, k-1);
    surgicalTip_3D_norm(:, k) = surgicalTip_3D_norm(:, k-1);
    elapsed_2(k) = toc; %End find tip timer
else
    [point3d_1,point3d_2, point3d_3] = findWorldCoordinates(centroidLeft,centroidRight,stereoParams);
    [surgicalTip_3D(:, k), rotMatrix] = findSurgicalTip(point3d_1,point3d_2,point3d_3,pivotOffset);
    if k > 10
        [surgicalTip_3D_norm(:,k)] = weightedAverage(surgicalTip_3D(:,:), k);
    end
    elapsed_2(k) = toc; %End find tip timer

    %Plotting in video player
    rgb = insertShape(frameLeft,'rectangle',bboxLeft(1,:),'Color','black',...
        'LineWidth',3);
    rgb = insertShape(rgb,'rectangle',bboxLeft(2,:),'Color','black',...
        'LineWidth',3);
    rgb = insertShape(rgb,'rectangle',bboxLeft(3,:),'Color','black',...
        'LineWidth',3);
    
    rgb = insertText(rgb,centroidLeft(1,:) - 20,['X: ' num2str(round(point3d_1(1,1)),'%d')...
        ' Y: ' num2str(round(point3d_1(1,2)),'%d') ' Z: ' num2str(round(point3d_1(1,3)))],'FontSize',18);
    rgb = insertText(rgb,centroidLeft(2,:) + 50,['X: ' num2str(round(point3d_2(1,1)),'%d')...
        ' Y: ' num2str(round(point3d_2(1,2)),'%d') ' Z: ' num2str(round(point3d_2(1,3)))],'FontSize',18);
    rgb = insertText(rgb,centroidLeft(3,:) - 50,['X: ' num2str(round(point3d_3(1,1)),'%d')...
        ' Y: ' num2str(round(point3d_3(1,2)),'%d') ' Z: ' num2str(round(point3d_3(1,3)))],'FontSize',18);
    
    rotMatrix;
    eul = rotm2eul(rotMatrix);
    player(rgb);
    writeVideo(v,rgb);
end

%Start world2microscope timer
tic;  

%Find location in microscope coordinates
[xMicroscope, yMicroscope, zMicroscope] = world2Microscope_Accuracy(surgicalTip_3D_norm(1, k), surgicalTip_3D_norm(2, k), surgicalTip_3D_norm(3, k), x_origin, y_origin, z_origin); %World to Microscope Coordinate Mapping

%End world2microscope timer
elapsed_3(k) = toc;

%Start control system timer
tic;

%Initiate control system
[q0,X,Y,Z,Q(count*10 - 9:count*10, :)] = moveMicroscope(xMicroscope, yMicroscope, zMicroscope, q0, Robot,eul);
count = count + 1;

%End control system timer
elapsed_4(k) = toc;

%Log control system accuracy
Robot_Accuracy(:,k) = [X,Y,Z];
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

[T, Equiv_FPS_Rate] = systemPerformance(elapsed_1,elapsed_2, elapsed_3, elapsed_4);
disp(T);
fprintf('Equivalent FPS Rate: %3.2f \n', Equiv_FPS_Rate);

%% Output Accuracy Metrics
%Vert 50
TAcc = trackingAccuracy(surgicalTip_3D_norm(2,:),50,Robot_Accuracy(3,:))
disp(TAcc);

figure (2);
subplot(331)
plot(surgicalTip_3D(1,12:235));
title('Surgical Tip Position X');
subplot(332)
plot(surgicalTip_3D_norm(1,12:235));
title('Normalized Surgical Tip Position X');
subplot(333)
plot(Robot_Accuracy(2,12:235));
title('Robot Effector Tip Position Y (Tracking X)');
subplot(334)
plot(surgicalTip_3D(2,12:235));
title('Surgical Tip Position Y');
subplot(335)
plot(surgicalTip_3D_norm(2,12:235));
title('Normalized Surgical Tip Position Y');
subplot(336)
plot(Robot_Accuracy(3,12:235));
title('Robot Effector Tip Position Z (Tracking Y)');
subplot(337)
plot(surgicalTip_3D(3,12:235));
title('Surgical Tip Position Z');
subplot(338)
plot(surgicalTip_3D_norm(3,12:235));
title('Normalized Surgical Tip Position Z');
subplot(339)
plot(Robot_Accuracy(1,12:235));
title('Normalized Surgical Tip Position X (Tracking Z)');
