%% Gray tracking 

addpath(genpath('Trial 18-19'));
load("stereoParams18.mat")

readerLeft= VideoReader('myVideoLeftPivot7.avi');
readerRight = VideoReader('myVideoLeftPivot7.avi');

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

point3d_coord(235,3) = 0;
count = 1;

while hasFrame(readerLeft) && hasFrame(readerRight)
    
frameLeft = readFrame(readerLeft);
frameLeftGray = rgb2gray(frameLeft);

frameRight = readFrame(readerRight);
frameRightGray = rgb2gray(frameRight);

threshold = 245;

%left
img_left = frameLeftGray > threshold;%figure;imshow(img_cut)
BW_left = bwareafilt(img_left, 3); % Extract largest blob.
[centroidLeft,bboxLeft] = step(hblob,BW_left);

%right
img_right = frameRightGray > threshold;%figure;imshow(img_cut)
BW_right = bwareafilt(img_right, 3); % Extract largest blob.
[centroidRight,bboxRight] = step(hblob,BW_right);


%Triangulate for all three markers

point3d_1 = triangulate(centroidRight(1,:),centroidLeft(1,:),stereoParams18);
point3d_2 = triangulate(centroidRight(2,:),centroidLeft(2,:),stereoParams18);
point3d_3 = triangulate(centroidRight(3,:),centroidLeft(3,:),stereoParams18);

%Store marker coordinates into frame
point3d_coord(count,1) = point3d_1(1,1);
point3d_coord(count,2) = point3d_1(1,2);
point3d_coord(count,3) = point3d_1(1,3);


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

[x_tip,y_tip,z_tip] = pivotCalibrate(point3d_coord);

tip_coord = [x_tip,y_tip,z_tip];

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