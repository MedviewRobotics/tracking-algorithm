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
close all;
addpath(genpath('Trial 18-19'));
load("stereoParams18.mat");

pivotOffset = 200; % 20cm offset from midpoint btwn blue and green
threshold = 240; % Threshold for Grayscale 

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

%% Marker tracking and robot movement
close all;
tic; %Start timer
count = 1;
x_world_test = 0;
y_world_test = 0;
z_world_test = 0;
tester = 0;

player = vision.DeployableVideoPlayer('Location',[10,100]);
v = VideoWriter('pivot.avi');
v.FrameRate = 30;
open(v)

for k = 1:1:nFramesLeft

%Read Frames
frameLeft = mov(k).readerLeft ;
frameRight = mov(k).readerRight;

%Convert to Grayscale
frameLeftGray = rgb2gray(frameLeft);
frameRightGray = rgb2gray(frameRight);

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
  
% World to Microscope Coordinate Mapping
[xMicroscope, yMicroscope, zMicroscope] = world2Microscope(surgicalTip(1), surgicalTip(2), surgicalTip(3));

count = count + 1;
x_world_test(count) = xMicroscope;
y_world_test(count) = yMicroscope;
z_world_test(count) = zMicroscope;

% Send Coordinates to AT03 Robot
q0 = moveMicroscope(xMicroscope, yMicroscope, zMicroscope, q0, Robot);

player(rgb);
writeVideo(v,rgb); 
end

release(player)
close(v);  

toc; %Stop timer

%% Pivot Offset Visualization

figure
plot3((point3d_3(1,1)), (point3d_3(1,2)), (point3d_3(1,3)), 'ro');
hold on
plot3((point3d_2(1,1)), (point3d_2(1,2)), (point3d_2(1,3)), 'bo');
plot3((point3d_1(1,1)), (point3d_1(1,2)), (point3d_1(1,3)), 'go');
plot3((surgicalTip(1,1)), (surgicalTip(1,2)), (surgicalTip(1,3)), 'co');
hold off