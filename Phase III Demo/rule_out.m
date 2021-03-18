
addpath(genpath('Accuracy Trials'));
load("stereoParamsAccuracy.mat");
threshold = 250;
player = vision.DeployableVideoPlayer('Location',[10,100]);
readerLeft = VideoReader('myLeftTrialDepth5cm.avi');
readerRight = VideoReader('myRightTrialHoriz5cm.avi');

hblob = vision.BlobAnalysis('AreaOutputPort', false, ... 
                                'CentroidOutputPort', true, ... 
                                'BoundingBoxOutputPort', true', ...
                                'MinimumBlobArea', 1, ...
                                'MaximumBlobArea', 20000, ...
                                'MaximumCount',3);
%%

markerLocations(1:235,2) = zeros;
count = 1;
warningCount = 1;
while hasFrame(readerRight)
%Read Frames
frameLeft = readFrame(readerRight);

% frameRight = readFrame(readerRight);

%Convert to Grayscale
frameLeftGray = rgb2gray(frameLeft);
frameLeftGray = imgaussfilt(frameLeftGray);
%frameLeftGray = medfilt2(frameLeftGray,[3 3]);
frameLeftGray = imsharpen(frameLeftGray);
% frameRightGray = rgb2gray(frameRight);

[M,N] = size(frameLeftGray);
%figure;imshow(diffFrameColor)
frameLeftGray(1:M,[1:0.3*N 0.65*N:N],:)=0;
frameLeftGray([1:200 550:M],1:N,:)=0;

%Detect markers in the Left 
img_left = frameLeftGray > threshold; %Creates binary image
img_left = bwareaopen(img_left, 22);
%figure;imshow(img_left);title('pre-dilation')
img_left = imdilate(img_left,strel('disk',5));
%filter out pixels below 200
%img_left = bwareaopen(img_left, 200);

cc = bwconncomp(img_left);
stats = regionprops(cc,'Area','Centroid','BoundingBox','Eccentricity','Circularity','Extent','EquivDiameter'); 
idx = find([stats.Area] > 190 & [stats.Area] < 550 & [stats.Eccentricity] > 0.2 & [stats.Eccentricity] < 0.65);% & ...
    %[stats.Circularity] > 1 & [stats.Circularity] < 1.2 & [stats.Extent] > 0.7 );
% idx = find([stats.Area] > 200 & [stats.Area] < 450 & [stats.Eccentricity] > 0.35 & [stats.Eccentricity] < 0.85 );%& [stats.Circularity] > 0.7 & [stats.Circularity] < 1.3);
BW2 = ismember(labelmatrix(cc),idx);  

%[centersBright, ~] = imfindcircles(img_left,[10 30],'ObjectPolarity','bright');
%BW_left = bwareafilt(BW2,3,'largest'); %Extract 3 largest blobs

[centroidLeft,bboxLeft] = step(hblob,BW2);
% 
% %store marker locations 
% markerLocations(count,1) = centroidLeft(1,1);
% markerLocations(count,2) = centroidLeft(1,2);


% %Detect markers in the Right
% img_right = frameRightGray > threshold; %Creates binary image
% BW_right = bwareafilt(img_right, 3); %Extract 3 largest blobs
% [centroidRight,bboxRight] = step(hblob,BW_right);

% Insert shape on markers
try
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
catch
    warning(['Marker not found in Frame' num2str(warningCount)]);
    warningCount = warningCount + 1;
    %figure;imshow(img_left)
end

pause(0.1)
player(rgb)

count = count + 1;

end
release(player)
disp(['accuracy = ' num2str(100 - (warningCount/235)*100)])
% [centersBright, radiiBright] = imfindcircles(img_left,[9 25],'ObjectPolarity','bright');
%viscircles(centersBright, radiiBright,'LineStyle','--');
% 
%% 
% RULE OUT SYSTEM
% cc = bwconncomp(img_left);
% stats = regionprops(cc,'Area','Centroid','BoundingBox','Eccentricity','Circularity','Extent','EquivDiameter'); 
% idx = find([stats.Area] > 250 & [stats.Area] < 450 & [stats.Eccentricity] > 0.35 & [stats.Eccentricity] < 0.6 &...
%     [stats.Circularity] > 0.9 & [stats.Circularity] < 1.2) 
% BW2 = ismember(labelmatrix(cc),idx);  
% 
% [centroidTEST,bboxTEST] = step(hblob,BW2)
% 
% figure;imshow(BW2)
% bw = bwareaopen(img_left,220);figure;imshow(bw)
% 
% 
% %% Euclidean Distance rule out system
% 
% cc = bwconncomp(img_left);
% stats = regionprops(cc,'All');
% a = [];
% a = vertcat(a,stats.Centroid);
% % %b = a(1,:); c = a(2,:);
% % %norm(b-c)
% % 
% for i = 1:length(a)
%     for j = 1:length(a)
%         dist(i,j) = norm(a(i,:)-a(j,:));
%     end
% end
% [row,col] = find(dist > 33 & dist < 45);
% 
% 
% %%
% 
% readerLeft = VideoReader('myLeftTrialVert10cm.avi');
% 
% frameLeft = readFrame(readerLeft);
% frameLeftGray = rgb2gray(frameLeft);
% frameLeftGray = imgaussfilt(frameLeftGray);
% frameLeftGray = imsharpen(frameLeftGray);
% % frameRightGray = rgb2gray(frameRight);
% figure;imshow(frameLeftGray)
% 
% [M,N] = size(frameLeftGray);
% %figure;imshow(diffFrameColor)
% frameLeftGray(1:M,[1:0.3*N 0.55*N:N],:)=0;
% frameLeftGray([1:200 550:M],1:N,:)=0;
% figure;imshow(frameLeftGray)
% 
% 
% 

%BW = bwareaopen(img_left, 260);figure;imshow(BW)

%%

[bw2,centroids,diffFrameColor] = detectmarkerColorDark(frameLeft,250,1,5);
figure;imshow(bw2)




