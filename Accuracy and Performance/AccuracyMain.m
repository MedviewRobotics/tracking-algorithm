%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% BME70B: Biomedical Engineering Capstone Design
% AT03 Robotic Automated Microscopy Marker Tracking and Robot Movement
% Phase II Demonstration
%
% Author #1: Ginette Hartell - 500755250
% Author #2: Mohammad Aziz Uddin - 500754765
% Author #3: Jay Tailor - 500750496
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

% readerLeft = VideoReader('myLeftTrialHoriz10cm.avi');
% readerRight = VideoReader('myRightTrialHoriz10cm.avi');

readerLeft = VideoReader('myLeftTrialDepth5cm.avi');
readerRight = VideoReader('myRightTrialDepth5cm.avi');

% readerLeft = VideoReader('myLeftTrialDepth5cm.avi');
% readerRight = VideoReader('myRightTrialDepth5cm.avi');

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
disp('Initialization Completed.');

elapsed_initialized = toc; %Assign toc to initialization time

%% Marker tracking and robot movement
close all;
clear surgicalTip_Accuracy;
clear Robot_Accuracy;

%Initialize Arrays
surgicalTip_3D = zeros(3,235);
surgicalTip_3D_norm = zeros(3,235);
surgicalTip_3D_normdev = zeros(3,235);
deviation = zeros(3,235);
Robot_Accuracy = zeros(3,235);
elapsed_1 = zeros(1, 235);
elapsed_2 = zeros(1, 235);
elapsed_3 = zeros(1, 235);
elapsed_4 = zeros(1, 235);

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

elapsed_1(k) = toc; %End preprocessing phase

tic; %Start timer for finding tip

[centroidLeft, bboxLeft, centroidRight, bboxRight] = ...
    findCentroids(frameLeftGray,frameRightGray,threshold,hblob);

if size(centroidLeft) ~= [3 3] | size(centroidRight) ~= [3 3]
    warning(['Could not find marker(s) in frame: ', num2str(k)])
    surgicalTip_3D(:, k) = surgicalTip_3D(:, k-1);
    surgicalTip_3D_norm(:, k) = surgicalTip_3D(:, k);
    elapsed_2(k) = toc; %End find tip timer
else
    [point3d_1,point3d_2, point3d_3] = findWorldCoordinates(centroidLeft,centroidRight,stereoParams);
    [surgicalTip_3D(:, k), rotMatrix] = findSurgicalTip(point3d_1,point3d_2,point3d_3,pivotOffset);
    if k > 5
        surgicalTip_3D_norm(1, k) = mean(surgicalTip_3D(1, k-5:k));
        surgicalTip_3D_norm(2, k) = mean(surgicalTip_3D(2, k-5:k));
        surgicalTip_3D_norm(3, k) = mean(surgicalTip_3D(3, k-5:k));
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
    
    player(rgb);
    writeVideo(v,rgb);
end

tic; %Start world2microscope timer
[xMicroscope, yMicroscope, zMicroscope] = world2Microscope_Accuracy(surgicalTip_3D_norm(1, k), surgicalTip_3D_norm(2, k), surgicalTip_3D_norm(3, k)); %World to Microscope Coordinate Mapping
elapsed_3(k) = toc;

tic; %Start control system timer
[xMicroscope, yMicroscope, zMicroscope] = safetyProtocols(xMicroscope, yMicroscope, zMicroscope); %Implementation of Safety Protocols
[q0,X,Y,Z] = moveMicroscope(xMicroscope, yMicroscope, zMicroscope, q0, Robot); %Send Coordinates to AT03 Robot
elapsed_4(k) = toc;

Robot_Accuracy(:,k) = [X,Y,Z];
end

release(player)
close(v);  

%% Output performance metrics

[T, Equiv_FPS_Rate] = systemPerformance(elapsed_1,elapsed_2, elapsed_3, elapsed_4);

disp(T);
fprintf('Equivalent FPS Rate: %3.2f \n', Equiv_FPS_Rate);

%% Output Accuracy Metrics
%Vert 50
TAcc = trackingAccuracy(surgicalTip_3D_norm(3,:),50,Robot_Accuracy(2,:));

figure;
subplot(211)
plot(surgicalTip_3D(2,6:235));
title('Surgical Tip Position');
subplot(212)
plot(surgicalTip_3D_norm(2,6:235));
title('Normalized Surgical Tip Position');

disp(TAcc);




