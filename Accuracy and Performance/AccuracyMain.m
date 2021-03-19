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

readerLeft = VideoReader('myLeftTrialHoriz10cm.avi');
readerRight = VideoReader('myRightTrialHoriz10cm.avi');

% readerLeft = VideoReader('myLeftTrialVert5cm.avi');
% readerRight = VideoReader('myRightTrialVert5cm.avi');

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

surgicalTip_Accuracy = zeros(3,235);
Robot_Accuracy = zeros(3,235);

%Initialize Arrays
elapsed_1 = zeros(1, 235);
elapsed_2 = zeros(1, 235);
elapsed_3 = zeros(1, 235);
elapsed_4 = zeros(1, 235);

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

%Initialize Video Player
player = vision.DeployableVideoPlayer('Location',[10,100]);
v = VideoWriter('accuracy.avi');
v.FrameRate = 30;
open(v)

frames_skip = 5;

for k = 1:frames_skip:nFramesLeft
tic %Starts pre-processing timer

%Read Frames
frameLeft = mov(k).readerLeft ;
frameRight = mov(k).readerRight;

%Pre-process Left and Gray Images
frameLeftGray = rgb2gray(frameLeft);
frameLeftGray = imgaussfilt(frameLeftGray);
frameLeftGray = imsharpen(frameLeftGray);

frameRightGray = rgb2gray(frameRight);
frameRightGray = imgaussfilt(frameRightGray);
frameRightGray = imsharpen(frameRightGray);

%Isolate Instrument Area
[M,N] = size(frameLeftGray);
frameLeftGray(1:M,[1:0.3*N 0.65*N:N],:)=0;
frameLeftGray([1:200 550:M],1:N,:)=0;

frameRightGray(1:M,[1:0.3*N 0.65*N:N],:)=0;
frameRightGray([1:200 550:M],1:N,:)=0;


elapsed_1(k) = toc; %End preprocessing phase
tic; %Start timer for finding tip

%Detect markers in the Left 
img_left = frameLeftGray > threshold; %Creates binary image
img_left = bwareaopen(img_left, 22);
img_left = imerode(img_left,strel('disk',1));
img_left = imdilate(img_left,strel('disk',6));

%Detect markers in the Right
img_right = frameRightGray > threshold; %Creates binary image
img_right = bwareaopen(img_right, 22);
img_right = imerode(img_right,strel('disk',1));
img_right = imdilate(img_right,strel('disk',6));

%Find features of the blobs and filter out any that don't meet criteria
cc_left = bwconncomp(img_left);
stats_left = regionprops(cc_left,'Area','Centroid','BoundingBox','Eccentricity','Circularity','Extent','EquivDiameter'); 
idx_left = find([stats_left.Area] > 165 & [stats_left.Area] < 550 & [stats_left.Eccentricity] > 0.1 & [stats_left.Eccentricity] < 0.67);% & ...
BW2_left = ismember(labelmatrix(cc_left),idx_left);  

cc_right = bwconncomp(img_right);
stats_right = regionprops(cc_right,'Area','Centroid','BoundingBox','Eccentricity','Circularity','Extent','EquivDiameter'); 
idx_right = find([stats_right.Area] > 165 & [stats_right.Area] < 550 & [stats_right.Eccentricity] > 0.1 & [stats_right.Eccentricity] < 0.67);% & ...
BW2_right = ismember(labelmatrix(cc_right),idx_right);  

%Find Centroid and Bounding Box of each marker
[centroidLeft,bboxLeft] = step(hblob,BW2_left);
[centroidRight,bboxRight] = step(hblob,BW2_right);

try
    %Triangulate for all three markers. R,G,B
    point3d_1 = triangulate(centroidLeft(1,:),centroidRight(1,:),stereoParams);
    point3d_2 = triangulate(centroidLeft(2,:),centroidRight(2,:),stereoParams);
    point3d_3 = triangulate(centroidLeft(3,:),centroidRight(3,:),stereoParams);
    
    %Find surgical tip location
    [surgicalTip_3D, rotMatrix] = findSurgicalTip(point3d_1,point3d_2,point3d_3,pivotOffset);
catch
end
surgicalTip_Accuracy(1:3,k) = surgicalTip_3D;
elapsed_2(k) = toc; %end find tip timer
try
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
    writeVideo(v,rgb);   
   
catch
    surgicalTip_Accuracy(1:3,k) = surgicalTip_Accuracy(1:3,k-1);
    surgicalTip_3D = surgicalTip_Accuracy(1:3,k-1);
end

count = count + 1;
x_world_test_new(count) = surgicalTip_3D(1);
y_world_test_new(count) = surgicalTip_3D(2);
z_world_test_new(count) = surgicalTip_3D(3);

tic; %start world2microscope timer
[xMicroscope, yMicroscope, zMicroscope] = world2Microscope_Accuracy(surgicalTip_3D(1), surgicalTip_3D(2), surgicalTip_3D(3)); %World to Microscope Coordinate Mapping
elapsed_3(k) = toc;

x_world_test_micro(count) = xMicroscope;
y_world_test_micro(count) = yMicroscope;
z_world_test_micro(count) = zMicroscope;

tic; %start control system timer
[xMicroscope, yMicroscope, zMicroscope] = safetyprotocols(xMicroscope, yMicroscope, zMicroscope); %Implementation of Safety Protocols

x_world_test_micro_2(count) = xMicroscope;
y_world_test_micro_2(count) = yMicroscope;
z_world_test_micro_2(count) = zMicroscope;

[q0,X,Y,Z] = moveMicroscope(xMicroscope, yMicroscope, zMicroscope, q0, Robot); %Send Coordinates to AT03 Robot
elapsed_4(k) = toc;

Robot_Accuracy(1:3,k) = [X,Y,Z];
end

release(player)
close(v);  
%% TS

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

%% Output performance metrics

[T, Equiv_FPS_Rate] = systemPerformance(elapsed_1,elapsed_2, elapsed_3, elapsed_4);

disp(T);
fprintf('Equivalent FPS Rate: %3.2f \n', Equiv_FPS_Rate);

%% Output Accuracy Metrics
%Horizontal 100
TAcc = trackingAccuracy(surgicalTip_Accuracy(1,:),100,Robot_Accuracy(2,:));
plot(surgicalTip_Accuracy(1,:));

%Vertical 50
% TAcc = trackingAccuracy(surgicalTip_Accuracy(2,:),100,Robot_Accuracy(3,:));
%plot(surgicalTip_Accuracy(2,:))

%Depth 50
% TAcc = trackingAccuracy(surgicalTip_Accuracy(3,:),100,Robot_Accuracy(1,:));
%plot(surgicalTip_Accuracy(3,:))

disp(TAcc);




