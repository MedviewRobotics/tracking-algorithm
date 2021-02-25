%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% BME70B: Biomedical Engineering Capstone Design
% AT03 Robotic Automated Microscopy Calibration and Tracking
% Phase I Demonstration
%
% Author #1: Ginette Hartell - 500755250
% Author #2: Mohammad Aziz Uddin - 500754765
% Author #3: Jay Tailor - 500750496
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialize Peter Corke's Toolbox (Run only once)
petercorkeinitialize();

%% Initialization
addpath(genpath('Trial 18-19'));
load("stereoParams18.mat");

readerLeft= VideoReader('myVideoLeftTrial18.avi');
readerRight = VideoReader('myVideoRightTrial18.avi');

player = vision.DeployableVideoPlayer('Location',[10,100]);
% v = VideoWriter('robottrack.avi');
% v.FrameRate = 30;
% open(v)

hblob = vision.BlobAnalysis('AreaOutputPort', false, ... % Set blob analysis handling
                                'CentroidOutputPort', true, ... 
                                'BoundingBoxOutputPort', true', ...
                                'MinimumBlobArea', 1, ...
                                'MaximumBlobArea', 20000, ...
                                'MaximumCount',3);

x_frame1 = -36; y_frame1=-80; z_frame1=30;
[Robot,q0] = initializeMicroscope(x_frame1,y_frame1,z_frame1);
%test

%% Marker tracking and robot movement

%Start timer
tic;

while hasFrame(readerLeft) && hasFrame(readerRight)

%Left Frame
frameLeft = readFrame(readerLeft);
frameLeftGray = rgb2gray(frameLeft);

%Right Frame
frameRight = readFrame(readerRight);
frameRightGray = rgb2gray(frameRight);

%Threshold for Grayscale 
threshold = 240;

%Detect markers in the Left 
img_left = frameLeftGray > threshold;%figure;imshow(img_cut)
BW_left = bwareafilt(img_left, 3); % Extract largest blob.
[centroidLeft,bboxLeft] = step(hblob,BW_left);

%Detect markers in the Right
img_right = frameRightGray > threshold;%figure;imshow(img_cut)
BW_right = bwareafilt(img_right, 3); % Extract largest blob.
[centroidRight,bboxRight] = step(hblob,BW_right);

%Triangulate for all three markers

point3d_1 = triangulate(centroidLeft(1,:),centroidRight(1,:),stereoParams18);
point3d_2 = triangulate(centroidLeft(2,:),centroidRight(2,:),stereoParams18);
point3d_3 = triangulate(centroidLeft(3,:),centroidRight(3,:),stereoParams18);

% Insert shape on markers
rgb = insertShape(frameLeft,'rectangle',bboxLeft(1,:),'Color','black',...
'LineWidth',3);
%GREEN
rgb = insertShape(rgb,'rectangle',bboxLeft(2,:),'Color','black',...
'LineWidth',3);
%BLUE
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

%count = count + 1;  
% World to Microscope Coordinate Mapping (for blue coordinates)
[xMicroscope, yMicroscope, zMicroscope] = world2Microscope(point3d_1(1), point3d_1(2), point3d_1(3))

% Send to Control System
% q0 = moveMicroscope(-36, 0, 113, q0, Robot);
q0 = moveMicroscope(xMicroscope, yMicroscope, zMicroscope, q0, Robot);
%q0 = moveMicroscope(-36, yMicroscope, 104, q0, Robot);
 
end

% release(player)
% close(v);  


% Send to Control System
% q0 = moveMicroscope(xMicroscope, yMicroscope, zMicroscope, q0, Robot);

%Stop timer
toc;

