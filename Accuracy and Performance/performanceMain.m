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

%addpath(genpath('Accuracy Trials'));
%load("stereoParamsAccuracy.mat");
addpath(genpath('Trial 18-19'));
load("stereoParams18.mat");

pivotOffset = 200; % 20cm offset from midpoint btwn blue and green
threshold = 250; % Threshold for Grayscale 

readerLeft = VideoReader('myVideoLeftTrial18.avi');
readerRight = VideoReader('myVideoRightTrial18.avi');

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

for k = 1:1:nFramesLeft
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

%Triangulate for all three markers
point3d_1 = triangulate(centroidLeft(1,:),centroidRight(1,:),stereoParams18);
point3d_2 = triangulate(centroidLeft(2,:),centroidRight(2,:),stereoParams18);
point3d_3 = triangulate(centroidLeft(3,:),centroidRight(3,:),stereoParams18);

%Find surgical tip location
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

tic; %start world2microscope timer
[xMicroscope, yMicroscope, zMicroscope] = world2Microscope(surgicalTip(1), surgicalTip(2), surgicalTip(3));%World to Microscope Coordinate Mapping
elapsed_3(k) = toc;

tic; %start control system timer
q0 = moveMicroscope(xMicroscope, yMicroscope, zMicroscope, q0, Robot); %Send Coordinates to AT03 Robot
elapsed_4(k) = toc;

player(rgb);
writeVideo(v,rgb); 
end

release(player)
close(v);  

%% Output performance metrics

[T, Equiv_FPS_Rate] = systemPerformance(elapsed_1,elapsed_2, elapsed_3, elapsed_4);

disp(T);
fprintf('Equivalent FPS Rate: %3.2f \n', Equiv_FPS_Rate);

%% [Delete Later] Plot the elapsed times
figure
subplot(4, 1, 1)
plot(elapsed_1)
title('Preprocessing')
subplot(4, 1, 2)
plot(elapsed_2)
title('Find Tip')
subplot(4, 1, 3)
plot(elapsed_3)
title('World2Microscope')
subplot(4, 1, 4)
plot(elapsed_4)
title('Control System')

%% [Delete Later] Plotting Surgical Tip plane
[surgicalTip, normal, planefunction, zplane] = findSurgicalTip(point3d_1,point3d_2,point3d_3, pivotOffset);

figure
fmesh(zplane)
hold on
plot3((point3d_3(1,1)), (point3d_3(1,2)), (point3d_3(1,3)), 'ro');
plot3((point3d_2(1,1)), (point3d_2(1,2)), (point3d_2(1,3)), 'bo');
plot3((point3d_1(1,1)), (point3d_1(1,2)), (point3d_1(1,3)), 'go');
plot3((surgicalTip(1,1)), (surgicalTip(1,2)), (surgicalTip(1,3)), 'mo');
hold off

