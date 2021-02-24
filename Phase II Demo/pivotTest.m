%% PIVOT TESTING

addpath(genpath('Trial 18-19'));
load("stereoParams18.mat")
readerLeft= VideoReader('myVideoLeftPivot7.avi');
readerRight= VideoReader('myVideoRightPivot7.avi');
% 
% 
% %% Find the markers
% 
% % player = vision.DeployableVideoPlayer('Location',[10,100]);
% % v = VideoWriter('pivot');
% % v.FrameRate = 30;
% % open(v)
% 
% % redThresh = 0.20; % Threshold for red detection
% % greenThresh = 0.10; % Threshold for green detection
% % blueThresh = 0.244; % Threshold for blue detection
% % %radii for disk structural element in Image dilation (determined
% % %experimentally)
% % radius_red = 10;
% % radius_green = 10;
% % radius_blue = 10;
% % 
% % hblob = vision.BlobAnalysis('AreaOutputPort', false, ... % Set blob analysis handling
% %                                 'CentroidOutputPort', true, ... 
% %                                 'BoundingBoxOutputPort', true', ...
% %                                 'MinimumBlobArea', 1, ...
% %                                 'MaximumBlobArea', 20000, ...
% %                                 'MaximumCount',3);
% 
% % ^INITIALIZATIONS IN FUNCTION detectMarkerLeft.m and detectMarkerRight.m 
% 
% %% Detect markers
% 
% player = vision.DeployableVideoPlayer('Location',[10,100]);
% v = VideoWriter('pivot.avi');
% v.FrameRate = 30;
% open(v)
% 
% point3dRED_coord(235,3) = 0;
% count = 1;
% while hasFrame(readerLeft)  && hasFrame(readerRight)
% 
% frameLeft = readFrame(readerLeft);
% frameRight = readFrame(readerRight);
% 
% %Setect colored markers for each camera
% %LEFT
% % [binFrameRedLeft,redCentroidsLeft] = detectmarkerColor(frameLeft,redThresh,1,radius_red);
% % rgb = insertText(frameLeft,redCentroidsLeft(1,:),['X: ' num2str(round(redCentroidsLeft(1,1)),'%d')...
% % ' Y: ' num2str(round(redCentroidsLeft(1,2)),'%d')],'FontSize',18);
% % figure;imshowpair(rgb,binFrameRedLeft,'montage')
% % 
% % [binFrameRedRight,redCentroidsRight] = detectmarkerColor(frameRight,0.15,1,radius_red);
% % rgb = insertText(frameRight,redCentroidsRight(1,:),['X: ' num2str(round(redCentroidsRight(1,1)),'%d')...
% % ' Y: ' num2str(round(redCentroidsRight(1,2)),'%d')],'FontSize',18);
% % figure;imshowpair(rgb,binFrameRedRight,'montage')
% % 
% % 
% % 
% % [binFrameGreenLeft,greenCentroidsLeft] = detectmarkerColor(frameLeft,greenThresh,2,radius_green);
% % rgb = insertText(frameLeft,greenCentroidsLeft(1,:),['X: ' num2str(round(greenCentroidsLeft(1,1)),'%d')...
% % ' Y: ' num2str(round(greenCentroidsLeft(1,2)),'%d')],'FontSize',18);
% % figure;imshowpair(rgb,binFrameGreenLeft,'montage')
% % 
% % 
% % [binFrameBlueLeft,blueCentroidsLeft] = detectmarkerColor(frameLeft,blueThresh,3,radius_blue);
% % rgb = insertText(frameLeft,blueCentroidsLeft(1,:),['X: ' num2str(round(blueCentroidsLeft(1,1)),'%d')...
% % ' Y: ' num2str(round(blueCentroidsLeft(1,2)),'%d')],'FontSize',18);
% % figure;imshowpair(rgb,binFrameBlueLeft,'montage')
% 
% %LEFT
% [centroidRedLeft,bboxRedLeft,centroidGreenLeft,bboxGreenLeft,centroidBlueLeft,bboxBlueLeft] = detectMarkerLeft(frameLeft);
% 
% %RIGHT
% [centroidRedRight,bboxRedRight,centroidGreenRight,bboxGreenRight,centroidBlueRight,bboxBlueRight] = detectMarkerRight(frameRight);
% 
% 
% %Z IN WORLD COORDINATES FOR RED
% point3dRED = triangulate(centroidRedLeft(1,:),centroidRedRight(1,:),stereoParams18);
% 
% %store coordinates into array to be analyzed for pivot calibration
% point3dRED_coord(count,1) = point3dRED(1,1);
% point3dRED_coord(count,2) = point3dRED(1,2);
% point3dRED_coord(count,3) = point3dRED(1,3);
% 
% %Z IN WORLD COORDINATES FOR GREEN
% point3dGREEN = triangulate(centroidGreenLeft(1,:),centroidGreenRight(1,:),stereoParams18);
% 
% %Z IN WORLD COORDINATES FOR BLUE
% point3dBLUE = triangulate(centroidBlueLeft(1,:),centroidBlueRight(1,:),stereoParams18);
% 
% %RED
% rgb = insertShape(frameLeft,'rectangle',bboxRedLeft(1,:),'Color','red',...
% 'LineWidth',3);
% %GREEN
% rgb = insertShape(rgb,'rectangle',bboxGreenLeft(1,:),'Color','green',...
% 'LineWidth',3);
% %BLUE
% rgb = insertShape(rgb,'rectangle',bboxBlueLeft(1,:),'Color','blue',...
% 'LineWidth',3);
% 
% rgb = insertText(rgb,centroidRedLeft(1,:) + 20,['X: ' num2str(round(point3dRED(1,1)),'%d')...
% ' Y: ' num2str(round(point3dRED(1,2)),'%d') ' Z: ' num2str(round(point3dRED(1,3)))],'FontSize',18);
%                             
% count = count + 1;    
% 
% player(rgb);
% writeVideo(v,rgb);
% 
% end                       
%                             
% release(player)
% close(v);                            
%                             
% %Extract within range                             
%            % x coord                                                % y coord                  
% % inRange = bboxRedLeft(:,[1 3]) > 450 & bboxRedLeft(:,[1 3]) < 650 & bboxRedLeft(:,[2 4]) > 450 & bboxRedLeft(:,[2 4]) < 500                   
% %                             
% % BInRange = bboxRedLeft(inRange);
% %                             
% %%  
% img = rgb2gray(frameLeft);
% threshold = 240;
% img_cut = img > threshold;figure;imshow(img_cut)
% 
% [centers, radii, metric] = imfindcircles(img_cut,[5 30]);
% centersStrong5 = centers(1:5,:); 
% radiiStrong5 = radii(1:5);
% metricStrong5 = metric(1:5);
% viscircles(centersStrong5, radiiStrong5,'EdgeColor','b');
% 
% 
% %% 
% hblob = vision.BlobAnalysis('AreaOutputPort', false, ... % Set blob analysis handling
%                                 'CentroidOutputPort', true, ... 
%                                 'BoundingBoxOutputPort', true', ...
%                                 'MinimumBlobArea', 1, ...
%                                 'MaximumBlobArea', 20000, ...
%                                 'MaximumCount',3);
%                             
% [centroid,bbox] = step(hblob,img_cut);
% 
% rgb = insertShape(frameLeft,'rectangle',bbox(1,:),'Color','red',...
% 'LineWidth',3);
% %GREEN
% rgb = insertShape(rgb,'rectangle',bbox(2,:),'Color','green',...
% 'LineWidth',3);
% %BLUE
% rgb = insertShape(rgb,'rectangle',bbox(3,:),'Color','blue',...
% 'LineWidth',3);
% 
% % rgb = insertText(rgb,centroidRedLeft(1,:) + 20,['X: ' num2str(round(point3dRED(1,1)),'%d')...
% % ' Y: ' num2str(round(point3dRED(1,2)),'%d') ' Z: ' num2str(round(point3dRED(1,3)))],'FontSize',18);
% %                             
% 
% figure;imshow(rgb)
% 
% %%
% % Get all the blob properties.  Can only pass in originalImage in version R2008a and later.
% [labeledImage, numberOfBlobs] = bwlabel(img_cut);
% blobMeasurements = regionprops(labeledImage, 'area', 'Centroid');
% 
% BW = bwareafilt(img_cut, 3); % Extract largest blob.
% 
% %% USING GRAYSCALE THRESHOLDING
% 
% img = rgb2gray(frameLeft);
% threshold = 240;
% img_cut = img > threshold;figure;imshow(img_cut)
% BW = bwareafilt(img_cut, 3); % Extract largest blob.
% 
% [centroid,bbox] = step(hblob,BW);
% 
% rgb = insertShape(frameLeft,'rectangle',bbox(1,:),'Color','black',...
% 'LineWidth',3);
% %GREEN
% rgb = insertShape(rgb,'rectangle',bbox(2,:),'Color','black',...
% 'LineWidth',3);
% %BLUE
% rgb = insertShape(rgb,'rectangle',bbox(3,:),'Color','black',...
% 'LineWidth',3);
% 
% % rgb = insertText(rgb,centroidRedLeft(1,:) + 20,['X: ' num2str(round(point3dRED(1,1)),'%d')...
% % ' Y: ' num2str(round(point3dRED(1,2)),'%d') ' Z: ' num2str(round(point3dRED(1,3)))],'FontSize',18);
% %                             
% figure;imshow(rgb)


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
point3d_coord(235,3) = 0;
count = 1;

while hasFrame(readerLeft) && hasFrame(readerRight)
    
frameLeft = readFrame(readerLeft);
frameLeftGray = rgb2gray(frameLeft);

frameRight = readFrame(readerRight);
frameRightGray = rgb2gray(frameRight);

threshold = 240;

%right
img_right = frameRightGray > threshold;%figure;imshow(img_cut)
BW_right = bwareafilt(img_right, 3); % Extract largest blob.
[centroidRight,bboxRight] = step(hblob,BW_right);

%left
img_left = frameLeftGray > threshold;%figure;imshow(img_cut)
BW_left = bwareafilt(img_left, 3); % Extract largest blob.
[centroidLeft,bboxLeft] = step(hblob,BW_left);

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
[imagePoints,boardSize] = detectCheckerboardPoints('left_1.jpg');
squareSize = 22; % in millimeters
worldPoints = generateCheckerboardPoints(boardSize,squareSize);
zCoord = zeros(size(worldPoints,1),1);
worldPoints = [worldPoints zCoord];

cameraParams = estimateCameraParameters(imagePoints,worldPoints);
%imUndistorted = undistortImage(imOrig,cameraParams);
%[imagePoints,boardSize] = detectCheckerboardPoints(imUndistorted);
[R,t] = extrinsics(imagePoints,worldPoints,cameraParams);

%zCoord = zeros(size(worldPoints,1),1);

projectedPoints = worldToImage(cameraParams,R,t,worldPoints);
hold on
plot(projectedPoints(:,1),projectedPoints(:,2),'g*-');
legend('Projected points');
hold off

%% World to Pixel coordinate % not functioning yet

imagePoints = worldToImage(stereoParams18.CameraParameters1.Intrinsics,...
    stereoParams18.CameraParameters1.RotationMatrices,...
    stereoParams18.CameraParameters1.TranslationVectors,tip_coord);

















                            