%% Gray tracking 

addpath(genpath('Trial 18-19'));
load("stereoParams18.mat")

readerLeft= VideoReader('myVideoLeftPivot7.avi');
readerRight = VideoReader('myVideoRightPivot7.avi');

player = vision.DeployableVideoPlayer('Location',[10,100]);
v = VideoWriter('pivot.avi');
v.FrameRate = 30;
open(v)

hblob = vision.BlobAnalysis('AreaOutputPort', false, ... % Set blob analysis handling
                                'CentroidOutputPort', true, ... 
                                'BoundingBoxOutputPort', true', ...
                                'MinimumBlobArea', 1, ...
                                'MaximumBlobArea', 20000, ...
                                'MaximumCount',3);
%% Tracking using Grayscale thresholding

% readerLeft= VideoReader('myVideoLeftTrial18.avi');
% readerRight = VideoReader('myVideoRightTrial 18.avi');


point3d_coord_1(235,3) = 0;
point3d_coord_2(235,3) = 0;
point3d_coord_3(235,3) = 0;
count = 1;

while hasFrame(readerLeft) && hasFrame(readerRight)
    
frameLeft = readFrame(readerLeft);
frameLeftGray = rgb2gray(frameLeft);

frameRight = readFrame(readerRight);
frameRightGray = rgb2gray(frameRight);

threshold = 240;

%left
img_left = frameLeftGray > threshold;%figure;imshow(img_cut)
BW_left = bwareafilt(img_left, 3); % Extract largest blob.
[centroidLeft,bboxLeft] = step(hblob,BW_left);

%right
img_right = frameRightGray > threshold;%figure;imshow(img_cut)
BW_right = bwareafilt(img_right, 3); % Extract largest blob.
[centroidRight,bboxRight] = step(hblob,BW_right);

%Triangulate for all three markers

point3d_1 = triangulate(centroidLeft(1,:),centroidRight(1,:),stereoParams18);
point3d_2 = triangulate(centroidLeft(2,:),centroidRight(2,:),stereoParams18);
point3d_3 = triangulate(centroidLeft(3,:),centroidRight(3,:),stereoParams18);

%Store marker coordinates into frame
point3d_coord_1(count,1) = point3d_1(1,1);
point3d_coord_1(count,2) = point3d_1(1,2);
point3d_coord_1(count,3) = point3d_1(1,3);

point3d_coord_2(count,1) = point3d_2(1,1);
point3d_coord_2(count,2) = point3d_2(1,2);
point3d_coord_2(count,3) = point3d_2(1,3);

point3d_coord_3(count,1) = point3d_3(1,1);
point3d_coord_3(count,2) = point3d_3(1,2);
point3d_coord_3(count,3) = point3d_3(1,3);


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

count = count + 1;  
    
end

release(player)
close(v);  

[x_tip_1,y_tip_1,z_tip_1] = pivotCalibrate(point3d_coord_1);
[x_tip_2,y_tip_2,z_tip_2] = pivotCalibrate(point3d_coord_2);
[x_tip_3,y_tip_3,z_tip_3] = pivotCalibrate(point3d_coord_3);

%tip_coord = [x_tip,y_tip,z_tip];

figure
plot3((point3d_3(1,1)), (point3d_3(1,2)), (point3d_3(1,3)), 'ro');
hold on
plot3((point3d_2(1,1)), (point3d_2(1,2)), (point3d_2(1,3)), 'bo');
plot3((point3d_1(1,1)), (point3d_1(1,2)), (point3d_1(1,3)), 'go');
plot3(x_tip_3,z_tip_3, y_tip_3, 'mo');
%cam = plotCamera;
hold off

%% World to Pixel coordinate % not functioning yet

%imOrig = imread('left_1.jpg');
% [imagePoints,boardSize] = detectCheckerboardPoints('left_1.jpg');
% squareSize = 22; % in millimeters
% worldPoints = generateCheckerboardPoints(boardSize,squareSize);
% zCoord = zeros(size(worldPoints,1),1);
% worldPoints = [worldPoints zCoord];
% 
% cameraParams = estimateCameraParameters(imagePoints,worldPoints);
% %imUndistorted = undistortImage(imOrig,cameraParams);
% %[imagePoints,boardSize] = detectCheckerboardPoints(imUndistorted);
% [R,t] = extrinsics(imagePoints,worldPoints,cameraParams);
% 
% %zCoord = zeros(size(worldPoints,1),1);
% 
% projectedPoints = worldToImage(cameraParams,R,t,worldPoints);
% hold on
% plot(projectedPoints(:,1),projectedPoints(:,2),'g*-');
% legend('Projected points');
% hold off
% 
% %% World to Pixel coordinate % not functioning yet
% 
% imagePoints = worldToImage(stereoParams18.CameraParameters1.Intrinsics,...
%     stereoParams18.CameraParameters1.RotationMatrices,...
%     stereoParams18.CameraParameters1.TranslationVectors,tip_coord);

%% Testing Generic Pivot Offset

%let offset be 200mm from midpoint between blue and green markers in
%direction of red marker

pivotOffset = 200;

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

%Determine point X between blue and green marker
g2b = point3d_2 - point3d_1;
g2b_norm = g2b/norm(g2b);
midpoint = point3d_1 + g2b_norm*37.5;

%Find 2D surgical tip
surgicalTip = findSurgicalTip(centroidLeft(1,:),centroidLeft(2,:), centroidLeft(3,:), pivotOffset);

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

end


imshow(frameLeft)

surgicalTip = findSurgicalTip(point3d_1,point3d_3, point3d_2, pivotOffset)

%validation
surgicalTip_validate = triangulate([505, 442],[582, 446],stereoParams18)

figure
plot3((point3d_3(1,1)), (point3d_3(1,2)), (point3d_3(1,3)), 'ro');
hold on
plot3((point3d_2(1,1)), (point3d_2(1,2)), (point3d_2(1,3)), 'bo');
plot3((point3d_1(1,1)), (point3d_1(1,2)), (point3d_1(1,3)), 'go');
plot3((midpoint(1,1)), (midpoint(1,2)), (midpoint(1,3)), 'mo');
plot3((surgicalTip(1,1)), (surgicalTip(1,2)), (surgicalTip(1,3)), 'co');
%cam = plotCamera;
hold off

%Determine point X between blue and green marker
g2b = point3d_2 - point3d_1;
g2b_norm = g2b/norm(g2b);
midpoint = point3d_1 + g2b_norm*37.5;

%Find direction vector from midpoint to red marker
m2r = point3d_3 - midpoint;
m2r_norm = m2r/norm(m2r);

%Find location of surgical tip
surgicalTip = midpoint + m2r_norm*pivotOffset;

%Stop timer
toc;

