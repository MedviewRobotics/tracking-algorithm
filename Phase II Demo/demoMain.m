%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% BME70B: Biomedical Engineering Capstone Design
% AT03 Robotic Automated Microscopy Calibration and Tracking
% Phase I Demonstration
%
% Author #1: Ginette Hartell - 500755250
% Author #2: Mohammad Aziz Uddin - 500754765
% Author #3: Jay Tailor - 500750496
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Initialization
addpath(genpath('Phase 1'));

readerLeft= VideoReader('myVideoLeftTrial9.avi');
readerRight= VideoReader('myVideoRightTrial9.avi');

%[stereoParams, estimationErrors] = stereoCalibrate();

player = vision.DeployableVideoPlayer('Location',[10,100]);
v = VideoWriter('xyz_all_markers.mp4');
v.FrameRate = 30;
open(v)

redThresh = 0.24; % Threshold for red detection
greenThresh = 0.18; % Threshold for green detection
blueThresh = 0.244; % Threshold for blue detection
%radii for disk structural element in Image dilation (determined
%experimentally)
radius_red = 1;
radius_green = 10;
radius_blue = 1;

hblob = vision.BlobAnalysis('AreaOutputPort', false, ... % Set blob analysis handling
                                'CentroidOutputPort', true, ... 
                                'BoundingBoxOutputPort', true', ...
                                'MinimumBlobArea', 1, ...
                                'MaximumBlobArea', 20000, ...
                                'MaximumCount',3);

[Robot,q0] = initializeMicroscope();

%% Marker tracking and robot movement
player = vision.DeployableVideoPlayer('Location',[10,100]);
v = VideoWriter('xyz_all_markers.mp4');
v.FrameRate = 30;
open(v)
tic;
while hasFrame(readerLeft) && hasFrame(readerRight)

frameLeft = readFrame(readerLeft);
frameRight = readFrame(readerRight);

%Setect colored markers for each camera
%LEFT
[binFrameRedLeft,redCentroidsLeft] = detectmarkerColor(frameLeft,redThresh,1,radius_red);
[binFrameGreenLeft,greenCentroidsLeft] = detectmarkerColor(frameLeft,greenThresh,2,radius_green);
[binFrameBlueLeft,blueCentroidsLeft] = detectmarkerColor(frameLeft,blueThresh,3,radius_blue);
%RIGHT
[binFrameRedRight,redCentroidsRight] = detectmarkerColor(frameRight,redThresh,1,radius_red);
[binFrameGreenRight,greenCentroidsRight] = detectmarkerColor(frameRight,greenThresh,2,radius_green);
[binFrameBlueRight,blueCentroidsRight] = detectmarkerColor(frameRight,blueThresh,3,radius_blue);

%Blob Analysis of each color
%Left
[centroidRedLeft,bboxRedLeft] = step(hblob,binFrameRedLeft);
[centroidGreenLeft,bboxGreenLeft] = step(hblob,binFrameGreenLeft);
[centroidBlueLeft,bboxBlueLeft] = step(hblob,binFrameBlueLeft);
%Right
[centroidRedRight,bboxRedRight] = step(hblob,binFrameRedRight);
[centroidGreenRight,bboxGreenRight] = step(hblob,binFrameGreenRight);
[centroidBlueRight,bboxBlueRight] = step(hblob,binFrameBlueRight);

%Computing 3D Coordinates
point3dRED = triangulate(centroidRedLeft(1,:),centroidRedRight(1,:),stereoParams);
point3dGREEN = triangulate(centroidGreenLeft(1,:),centroidGreenRight(1,:),stereoParams);
point3dBLUE = triangulate(centroidBlueLeft(1,:),centroidBlueRight(1,:),stereoParams);

%Generating Bounding Boxes
rgb = insertShape(frameLeft,'rectangle',bboxRedLeft(1,:),'Color','red',...
'LineWidth',3);%Red
rgb = insertShape(rgb,'rectangle',bboxGreenLeft(1,:),'Color','green',...
'LineWidth',3);%Green
rgb = insertShape(rgb,'rectangle',bboxBlueLeft(1,:),'Color','blue',...
'LineWidth',3);%Blue

%Plotting output in world coordinates
rgb = insertText(rgb,centroidRedLeft(1,:) + 20,['X: ' num2str(round(point3dRED(1)),'%d')...
' Y: ' num2str(round(point3dRED(2)),'%d') ' Z: ' num2str(round(point3dRED(3)))],'FontSize',18);
rgb = insertText(rgb,centroidGreenLeft(1,:)+15,['X: ' num2str(round(point3dGREEN(1)),'%d')...
' Y: ' num2str(round(point3dGREEN(2)),'%d') ' Z: ' num2str(round(point3dGREEN(3)))] ,'FontSize',18);
rgb = insertText(rgb,centroidBlueLeft(1,:)-75,['X: ' num2str(round(point3dBLUE(1)),'%d')...
' Y: ' num2str(round(point3dBLUE(2)),'%d') ' Z: ' num2str(round(point3dBLUE(3)))],'FontSize',18);

player(rgb);
pause(0.2)
writeVideo(v,rgb);

% World to Microscope Coordinate Mapping (for blue coordinates)
[xMicroscope, yMicroscope, zMicroscope] = world2Microscope(point3dBLUE(1), point3dBLUE(2), point3dBLUE(3))

% Send to Control System
q0 = moveMicroscope(xMicroscope, yMicroscope, zMicroscope, q0, Robot);

end
toc;

% release(player)
% close(v);

