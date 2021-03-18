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
threshold = 250; % Threshold for Grayscale 

readerLeft = VideoReader('myLeftTrialNormalMovement.avi');
readerRight = VideoReader('myRightTrialNormalMovement.avi');

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
point3d_1_Accuracy(:,3) = zeros();

%Initialize Arrays
elapsed_1 = zeros(1, 235);
elapsed_2 = zeros(1, 235);
elapsed_3 = zeros(1, 235);
elapsed_4 = zeros(1, 235);

%Initialize Video Player
player = vision.DeployableVideoPlayer('Location',[10,100]);
v = VideoWriter('pivot.avi');
v.FrameRate = 30;
open(v)

count = 1;
x_world_test_new = 0;
y_world_test_new = 0;
z_world_test_new = 0;
x_world_test_micro = 0;
y_world_test_micro = 0;
z_world_test_micro = 0;
x_world_test_micro_2 = 0;
y_world_test_micro_2 = 0;
z_world_test_micro_2 = 0;
tester = 0;

frames_skip = 1;

for k = 1:frames_skip:nFramesLeft
tic %Starts pre-processing timer

%Read Frames
frameLeft = mov(k).readerLeft ;
frameRight = mov(k).readerRight;

%Convert to Grayscale
frameLeftGray = rgb2gray(frameLeft);
frameRightGray = rgb2gray(frameRight);

elapsed_1(k) = toc; %End preprocessing phase
tic; %Start timer for finding tip

%Detect markers in the Left 
img_left = frameLeftGray > threshold; %Creates binary image
BW_left = bwareafilt(img_left, 3); %Extract 3 largest blobs
[centroidLeft,bboxLeft] = step(hblob,BW_left);

%Detect markers in the Right
img_right = frameRightGray > threshold; %Creates binary image
BW_right = bwareafilt(img_right, 3); %Extract 3 largest blobs
[centroidRight,bboxRight] = step(hblob,BW_right);

%Triangulate for all three markers. R,G,B
point3d_1_Accuracy(k,:) = triangulate(centroidLeft(1,:),centroidRight(1,:),stereoParams);
point3d_1 = triangulate(centroidLeft(1,:),centroidRight(1,:),stereoParams);
point3d_2 = triangulate(centroidLeft(2,:),centroidRight(2,:),stereoParams);
point3d_3 = triangulate(centroidLeft(3,:),centroidRight(3,:),stereoParams);

%Find surgical tip location
surgicalTip_Accuracy(k,:) = findSurgicalTip(point3d_1,point3d_2,point3d_3, pivotOffset);
surgicalTip = findSurgicalTip(point3d_1,point3d_2,point3d_3, pivotOffset);

elapsed_2(k) = toc; %end find tip timer

% Insert shape on markers
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

count = count + 1;
x_world_test_new(count) = surgicalTip(1);
y_world_test_new(count) = surgicalTip(2);
z_world_test_new(count) = surgicalTip(3);

tic; %start world2microscope timer
[xMicroscope, yMicroscope, zMicroscope] = world2Microscope(surgicalTip(1), surgicalTip(2), surgicalTip(3)); %World to Microscope Coordinate Mapping
elapsed_3(k) = toc;

x_world_test_micro(count) = xMicroscope;
y_world_test_micro(count) = yMicroscope;
z_world_test_micro(count) = zMicroscope;

[xMicroscope, yMicroscope, zMicroscope] = safetyprotocols(xMicroscope, yMicroscope, zMicroscope); %Implementation of Safety Protocols

x_world_test_micro_2(count) = xMicroscope;
y_world_test_micro_2(count) = yMicroscope;
z_world_test_micro_2(count) = zMicroscope;

tic; %start control system timer
q0 = moveMicroscope(xMicroscope, yMicroscope, zMicroscope, q0, Robot); %Send Coordinates to AT03 Robot
elapsed_4(k) = toc;

player(rgb);
writeVideo(v,rgb); 
end

release(player)
close(v);  

figure;
subplot(311)
plot(y_world_test_new);
title ('Surgical Tip Local Coordinates - "Y" from Tracking')
subplot(312)
plot(z_world_test_micro);
title ('Surgical Tip World Coordinates - "Z" for Ctrl Sys')
subplot(313)
plot(z_world_test_micro_2);
title ('Surgical Tip World Coordinates with Saftety Bounds - "Z" for Ctrl Sys')
xlabel('Frame #');

figure;
subplot(311)
plot(x_world_test_new);
title ('Surgical Tip Local Coordinates - "X" from Tracking')
subplot(312)
plot(y_world_test_micro);
title ('Surgical Tip World Coordinates - "Y" for Ctrl Sys')
subplot(313)
plot(y_world_test_micro_2);
title ('Surgical Tip World Coordinates with Saftety Bounds - "Y" for Ctrl Sys')
xlabel('Frame #');

figure;
subplot(311)
plot(z_world_test_new);
title ('Surgical Tip Local Coordinates - "Z" from Tracking')
subplot(312)
plot(x_world_test_micro);
title ('Surgical Tip World Coordinates - "X for Ctrl Sys')
subplot(313)
plot(x_world_test_micro_2);
title ('Surgical Tip World Coordinates with Saftety Bounds - "X" for Ctrl Sys')
xlabel('Frame #');

%% NOT DONE
%X for Jay %NO SHIFT! 
%If Z in Ginette increases ... Robot should move BACK (-x / into itself)
z_min = min(z_world_test_new); %1328.8 Outlier:0 %TRY 70
z_max = max(z_world_test_new); %1789 %TRY 20

%When ginette moves deepr in video (Depth/Z_word INCREASES) ... X value decreases in Robot

% Y in Jay
%cross section of image is ZERO
x_min = min(x_world_test_new); %-224 ... Outlier: -378 %TRY -40  
x_max = max(x_world_test_new); %-92.97 ... Outlier: 0  %TRY 40

% Ginette Y value goes UP as she moves DOWN in video
% robot should move DOWN when Ginette moves DOWN
% MEANS Robot z value decrease when ginette Y value increase
% cross section of image is ZERO
y_min = min(y_world_test_new); %-104                       %TRY 130
y_max = max(y_world_test_new); %-20 %her lowest Outlier:40 %TRY 100 


%% Output Accuracy Metrics

[Tracked_Displacement,Accuracy] = trackingAccuracy(point3d_1_Accuracy(:,1),100);
T = table(Tracked_Displacement, Accuracy);