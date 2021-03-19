
addpath(genpath('Accuracy Trials'));
load("stereoParamsAccuracy.mat");
threshold = 245;
player = vision.DeployableVideoPlayer('Location',[10,100]);

readerLeft = VideoReader('myLeftTrialHoriz5cm.avi');
readerRight = VideoReader('myRightTrialHoriz5cm.avi');

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

                                              
%%

markerLocations(1:235,2) = zeros;
count = 1;
warningCount = 1;
frameLeftGray_hist(720,1280,235) = zeros; 
frameLeftGray_hist_1(720,1280,235) = zeros; 

for k = 1:nFramesLeft
%Read Frames
frameLeft = mov(k).readerLeft;
frameRight = mov(k).readerRight;

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
%figure;imshow(img_left);title('pre-filter')
% cc_1 = bwconncomp(img_left);
% stats_1 = regionprops(cc_1,'Area','Centroid','BoundingBox','Eccentricity','Circularity','Extent','EquivDiameter'); 
% idx_1 = find([stats_1.Eccentricity] > 0.1 & [stats_1.Eccentricity] < 0.67);
% BW2_1 = ismember(labelmatrix(cc_1),idx);  
% figure;imshow(BW2_1);title('GET RID OF LINE')
frameLeftGray_hist_1(:,:,k) = img_left;
img_left = imerode(img_left,strel('disk',1));
img_left = imdilate(img_left,strel('disk',6));
%img_left = imerode(img_left,strel('disk',2));
frameLeftGray_hist(:,:,k) = img_left;
%filter out pixels below 200
%img_left = bwareaopen(img_left, 200);

cc = bwconncomp(img_left);
stats = regionprops(cc,'Area','Centroid','BoundingBox','Eccentricity','Circularity','Extent','EquivDiameter'); 
idx = find([stats.Area] > 165 & [stats.Area] < 550 & [stats.Eccentricity] > 0.1 & [stats.Eccentricity] < 0.67);% & ...
BW2 = ismember(labelmatrix(cc),idx);  
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
    warning(['Marker not found in Frame ' num2str(k)]);
    warningCount = warningCount + 1;
    disp([num2str([stats_1(:,:).Area stats_1.Eccentricity])])
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
%b = a(1,:); c = a(2,:);
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
% 
% [bw2,centroids,diffFrameColor] = detectmarkerColorDark(frameLeft,250,1,5);
% figure;imshow(bw2)


pre_dilate_img = frameLeftGray_hist_1(:,:,232);
post_dilate_img = frameLeftGray_hist(:,:,232);

figure;imshowpair(pre_dilate_img,post_dilate_img,'montage')

img_left_1 = imerode(pre_dilate_img,strel('disk',1));
img_left_2 = imdilate(img_left_1,strel('disk',6));

figure;montage({pre_dilate_img,img_left_1,img_left_2})

ccc = bwconncomp(img_left_2);
stats_1 = regionprops(ccc,'All');












