%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% BME70B: Biomedical Engineering Capstone Design
% AT03 Robotic Automated Microscopy Marker Tracking and Robot Movement
% Phase III Demonstration
%
% Author #1: Mohammad Aziz Uddin - 500754765
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialize Tracking System Parameters
addpath(genpath('Accuracy Trials'));
load("stereoParamsAccuracy.mat");
[mov,hblob,redThresh,greenThresh,blueThresh] = setupSystemObjects();
radius_red = 15;
radius_green = 15;
radius_blue = 15; 

%%
tic;
player = vision.DeployableVideoPlayer('Location',[10,100]);
readerLeft = VideoReader('myLeftTrialDepth5cm.avi');
readerRight = VideoReader('myRightTrialHoriz10cm.avi');
nFramesLeft = readerLeft.NumFrames;

while hasFrame(readerLeft)
    
    %Find markers
    image = readFrame(readerLeft);
    %image = imgaussfilt3(image);
    [binFrameRed,redCentroids] = detectmarkerColor(image,redThresh,1,radius_red);
    [binFrameGreen,greenCentroids] = detectmarkerColor(image,greenThresh,2,radius_green);
    [binFrameBlue,blueCentroids] = detectmarkerColor(image,blueThresh,3,radius_blue);
    
    %Blob Analysis of each color
    [centroidRed,bboxRed] = step(hblob,binFrameRed);
    [centroidGreen,bboxGreen] = step(hblob,binFrameGreen);
    [centroidBlue,bboxBlue] = step(hblob,binFrameBlue);

     %RED
     rgb = insertShape(image,'rectangle',bboxRed(1,:),'Color','red',...
         'LineWidth',3);
     %GREEN
     rgb = insertShape(rgb,'rectangle',bboxGreen(1,:),'Color','green',...
         'LineWidth',3);
     %BLUE
     rgb = insertShape(rgb,'rectangle',bboxBlue(1,:),'Color','blue',...
         'LineWidth',3);
     
     rgb = insertText(rgb,centroidRed(1,:) + 20,['X: ' num2str(round(centroidRed(1,1)),'%d')...
         ' Y: ' num2str(round(centroidRed(1,2)),'%d')],'FontSize',18);
     rgb = insertText(rgb,centroidGreen(1,:)+15,['X: ' num2str(round(centroidGreen(1,1)),'%d')...
         ' Y: ' num2str(round(centroidGreen(1,2)),'%d')],'FontSize',18);
     rgb = insertText(rgb,centroidBlue(1,:)-75,['X: ' num2str(round(centroidBlue(1,1)),'%d')...
         ' Y: ' num2str(round(centroidBlue(1,2)),'%d')],'FontSize',18);
     
     player(rgb);
     
end

release(player);

toc; %Stop timer


%%
radius_red = 1;
radius_green = 10;
radius_blue = 1; 

readerLeft = VideoReader('myLeftTrialHoriz10cm.avi');
readerRight = VideoReader('myRightTrialHoriz10cm.avi');
image = readFrame(readerLeft);

[binFrameRed,redCentroids] = detectmarkerColor(image,redThresh,1,radius_red);
[binFrameGreen,greenCentroids] = detectmarkerColor(image,greenThresh,2,radius_green);
[binFrameBlue,blueCentroids] = detectmarkerColor(image,blueThresh,3,radius_blue);

%Blob Analysis of each color
[centroidRed,bboxRed] = step(hblob,binFrameRed);
[centroidGreen,bboxGreen] = step(hblob,binFrameGreen);
[centroidBlue,bboxBlue] = step(hblob,binFrameBlue);

%RED
rgb = insertShape(image,'rectangle',bboxRed(1,:),'Color','red',...
'LineWidth',3);
%GREEN
rgb = insertShape(rgb,'rectangle',bboxGreen(1,:),'Color','green',...
'LineWidth',3);
%BLUE
rgb = insertShape(rgb,'rectangle',bboxBlue(1,:),'Color','blue',...
'LineWidth',3);

rgb = insertText(rgb,centroidRed(1,:) + 20,['X: ' num2str(round(centroidRed(1,1)),'%d')...
' Y: ' num2str(round(centroidRed(1,2)),'%d')],'FontSize',18);
rgb = insertText(rgb,centroidGreen(1,:)+15,['X: ' num2str(round(centroidGreen(1,1)),'%d')...
' Y: ' num2str(round(centroidGreen(1,2)),'%d')],'FontSize',18);
rgb = insertText(rgb,centroidBlue(1,:)-75,['X: ' num2str(round(centroidBlue(1,1)),'%d')...
' Y: ' num2str(round(centroidBlue(1,2)),'%d')],'FontSize',18);

%% Isolate where the instrument is by darkening the image
readerLeft = VideoReader('myLeftTrialHoriz10cm.avi');
readerRight = VideoReader('myRightTrialHoriz10cm.avi');
frameLeft = readFrame(readerLeft);
diffFrameColor= imsubtract(frameLeft(:,:,3), rgb2gray(frameLeft));
[M,N] = size(diffFrameColor);
figure;imshow(diffFrameColor)
%darken image to remove extra areas
diffFrameColor(1:M,[1:0.4*N 0.65*N:N],:)=0;
diffFrameColor([1:350 600:M],1:N,:)=0;
figure
imshow(diffFrameColor)
binFrameColor = imbinarize(diffFrameColor,0.2); 
%diffFrameBW = diffFrameColor > 7; 
figure;imshow(binFrameColor)

image = imdilate(binFrameColor, strel('disk',10));
bw2 = imfill(image,'holes');
figure
imshow(bw2)
title('Post processed image')


 frameLeft_1 = frameLeft;
% frameLeft_1(1:M,[1:0.3*N 0.55*N:N])=0;
% figure;imshow(frameLeft_1)

[M,N] = size(diffFrameBW);
diffFrameBW(1:M,0.55*N:N)=0;
figure
imshow(diffFrameBW)


%img1 = darkenImage(frameLeft);
frameLeft_1 = readFrame(readerLeft);
frameLeft_1(1:M,[1:0.4*N 0.65*N:N],:)=0;
frameLeft_1([1:350 600:M],1:N,:)=0;
figure;imshow(frameLeft_1)

figure;subplot(131);imshow(frameLeft);subplot(132);imshow(frameLeft_1);subplot(133);imshow(bw2)


%% USING IMAGE SUBTRACTION

readerLeft = VideoReader('myLeftTrialHoriz10cm.avi');
readerRight = VideoReader('myRightTrialHoriz10cm.avi');
player = vision.DeployableVideoPlayer('Location',[10,100]);

hblob = vision.BlobAnalysis('AreaOutputPort', false, ... 
                                'CentroidOutputPort', true, ... 
                                'BoundingBoxOutputPort', true', ...
                                'MinimumBlobArea', 1, ...
                                'MaximumBlobArea', 20000, ...
                                'MaximumCount',3);


while hasFrame(readerLeft) %&& hasFrame(readerRight)
   
frameLeft = readFrame(readerLeft);    
redLeft = detectmarkerColorDark(frameLeft,0.1,1,6);
greenLeft = detectmarkerColorDark(frameLeft,0.1,2,3);
blueLeft = detectmarkerColorDark(frameLeft,0.1,3,6);

    
%Blob Analysis of each color
[centroidRed,bboxRed] = step(hblob,redLeft);
[centroidGreen,bboxGreen] = step(hblob,greenLeft);
[centroidBlue,bboxBlue] = step(hblob,blueLeft);

%RED
rgb = insertShape(frameLeft,'rectangle',bboxRed(1,:),'Color','red',...
'LineWidth',3);
%GREEN
rgb = insertShape(rgb,'rectangle',bboxGreen(1,:),'Color','green',...
'LineWidth',3);
% %BLUE
rgb = insertShape(rgb,'rectangle',bboxBlue(1,:),'Color','blue',...
'LineWidth',3);

rgb = insertText(rgb,centroidRed(1,:) + 20,['X: ' num2str(round(centroidRed(1,1)),'%d')...
' Y: ' num2str(round(centroidRed(1,2)),'%d')],'FontSize',18);
rgb = insertText(rgb,centroidGreen(1,:)+15,['X: ' num2str(round(centroidGreen(1,1)),'%d')...
' Y: ' num2str(round(centroidGreen(1,2)),'%d')],'FontSize',18);
rgb = insertText(rgb,centroidBlue(1,:)-75,['X: ' num2str(round(centroidBlue(1,1)),'%d')...
' Y: ' num2str(round(centroidBlue(1,2)),'%d')],'FontSize',18); 

pause(0.5)
player(rgb)

end
release(player)


%% USING GREYSCALE THRESH

threshold = 250;
player = vision.DeployableVideoPlayer('Location',[10,100]);
readerLeft = VideoReader('myLeftTrialHoriz10cm.avi');

markerLocations(1:235,2) = zeros;
count = 1;
while hasFrame(readerLeft)
%Read Frames
frameLeft = readFrame(readerLeft);

% frameRight = readFrame(readerRight);

%Convert to Grayscale
frameLeftGray = rgb2gray(frameLeft);
frameLeftGray = imgaussfilt(frameLeftGray);
frameLeftGray = imsharpen(frameLeftGray);
% frameRightGray = rgb2gray(frameRight);

[M,N] = size(frameLeftGray);
%figure;imshow(diffFrameColor)
frameLeftGray(1:M,[1:0.4*N 0.65*N:N],:)=0;
frameLeftGray([1:350 600:M],1:N,:)=0;

%Detect markers in the Left 
img_left = frameLeftGray > threshold; %Creates binary image
img_left = imdilate(img_left,strel('disk',5));
cc = bwconncomp(img_left);
stats = regionprops(cc,'Area','Centroid','BoundingBox','Eccentricity','Circularity','Extent','EquivDiameter'); 
% idx = find([stats.Area] > 230 & [stats.Area] < 450 & [stats.Eccentricity] > 0.35 & [stats.Eccentricity] < 0.85 & ...
%     [stats.Circularity] > 0.7 & [stats.Circularity] < 1.5)
idx = find([stats.Area] > 230 & [stats.Area] < 450 & [stats.Circularity] > 0.7 & [stats.Circularity] < 1.3);
BW2 = ismember(labelmatrix(cc),idx);  

%[centersBright, ~] = imfindcircles(img_left,[10 30],'ObjectPolarity','bright');
BW_left = bwareafilt(BW2,3,'largest'); %Extract 3 largest blobs

[centroidLeft,bboxLeft] = step(hblob,BW_left);
% 
% %store marker locations 
% markerLocations(count,1) = centroidLeft(1,1);
% markerLocations(count,2) = centroidLeft(1,2);


% %Detect markers in the Right
% img_right = frameRightGray > threshold; %Creates binary image
% BW_right = bwareafilt(img_right, 3); %Extract 3 largest blobs
% [centroidRight,bboxRight] = step(hblob,BW_right);

% Insert shape on markers
rgb = insertShape(frameLeft,'rectangle',bboxLeft(1,:),'Color','black',...
'LineWidth',3);
rgb = insertShape(rgb,'rectangle',bboxLeft(2,:),'Color','black',...
'LineWidth',3);
rgb = insertShape(rgb,'rectangle',bboxLeft(3,:),'Color','black',...
'LineWidth',3);

rgb = insertText(rgb,centroidLeft(1,:) + 20,['X: ' num2str(round(centroidLeft(1,1)),'%d')...
' Y: ' num2str(round(centroidLeft(1,2)),'%d')],'FontSize',18);

rgb = insertText(rgb,centroidLeft(2,:)+ 20,['X: ' num2str(round(centroidLeft(2,1)),'%d')...
' Y: ' num2str(round(centroidLeft(2,2)),'%d')],'FontSize',18);

rgb = insertText(rgb,centroidLeft(3,:)+20,['X: ' num2str(round(centroidLeft(3,1)),'%d')...
' Y: ' num2str(round(centroidLeft(3,2)),'%d')],'FontSize',18); 

pause(0.1)
player(rgb)

count = count + 1;

end

release(player)

% [centersBright, radiiBright] = imfindcircles(img_left,[9 25],'ObjectPolarity','bright');
% viscircles(centersBright, radiiBright,'LineStyle','--');
% 
% 
% RULE OUT SYSTEM
cc = bwconncomp(img_left);
stats = regionprops(cc,'Area','Centroid','BoundingBox','Eccentricity','Circularity','Extent','EquivDiameter'); 
idx = find([stats.Area] > 250 & [stats.Area] < 450 & [stats.Eccentricity] > 0.35 & [stats.Eccentricity] < 0.6 &...
    [stats.Circularity] > 0.9 & [stats.Circularity] < 1.2) 
BW2 = ismember(labelmatrix(cc),idx);  

[centroidTEST,bboxTEST] = step(hblob,BW2)

figure;imshow(BW2)



