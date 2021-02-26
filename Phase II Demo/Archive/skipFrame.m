%% Initialization

addpath(genpath('Trial 18-19'));
load("stereoParams18.mat")

readerLeft = VideoReader('myVideoLeftTrial18.avi');
readerRight = VideoReader('myVideoRightTrial 18.avi');

nFramesLeft = readerLeft.NumFrames;  %Clip1.NumberOfFrames;
vidHeightLeft = readerLeft.Height;
vidWidthLeft = readerLeft.Width;

nFramesRight = readerRight.NumFrames;  %Clip1.NumberOfFrames;
vidHeightRight = readerRight.Height;
vidWidthRight = readerRight.Width;

%%Movie Structure
mov(1:nFramesLeft) = ...
    struct('readerLeft',zeros(vidHeightLeft,vidWidthLeft, 3,'uint8'),...
           'readerRight',zeros(vidHeightRight,vidWidthRight, 3,'uint8'),...
            'colormap',[]);

for k = 1:nFramesLeft
mov(k).readerLeft = read(readerLeft,k);
mov(k).readerRight = read(readerRight,k);
end

img = mov(5).readerLeft;
imshow(img)

%%
player = vision.DeployableVideoPlayer('Location',[10,100]);


for k = 1:10:nFramesLeft
% figure
% imshowpair(mov(k).readerLeft,mov(k).readerRight,'montage') 
% title(['Frame #: ' num2str(k)])

frameLeft = mov(k).readerLeft ;
frameRight = mov(k).readerRight;

frameLeftGray = rgb2gray(frameLeft);
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

release(player)










