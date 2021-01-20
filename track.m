% Stereovision object tracking and depth estimation

load('handshakeStereoParams.mat')
%% Show camera parameters
showExtrinsics(stereoParams)

%% Define left and right camera videos

videoFileLeft = 'handshake_left.avi'
videoFileRight = 'handshake_right.avi'

readerLeft = VideoReader(videoFileLeft)
readerRight = VideoReader(videoFileRight)
player = vision.VideoPlayer('Position', [20, 200, 740 560])

%% Read and Rectify Video Frames
% left and right video must be rectified to compute disparity and
% reconstruct the 3-D scene

frameLeft = readFrame (readerLeft)
frameRight = readFrame(readerRight)

[frameLeftRect,frameRightRect] = rectifyStereoImages(frameLeft,frameRight,...
    stereoParams)

figure;
imshow(stereoAnaglyph(frameLeftRect,frameRightRect))
title('Rectified Video Frames')

%% Compute Disparity
% match ecorresponding points in stereo images on same pixel row.
% For each pixel in the left image compute the distance to the corresponding 
% right image

frameLeftGray = rgb2gray(frameLeftRect);
frameRightGray = rgb2gray(frameRightRect);

disparityMap = disparitySGM(frameLeftGray,frameRightGray)
figure
imshow(disparityMap,[0 64]);
title('Disparity Map')
colormap jet
colorbar

%% 3-D Reconstruct scene

points3D = reconstructScene(disparityMap,stereoParams);


% Convert to meters and create a pointCloud object
points3D = points3D ./ 1000; % 
ptCloud = pointCloud(points3D,'Color',frameLeftRect)

% Create a streaming point cloud viewer
player3D = pcplayer([-3,3],[-3,3],[0,8],'VerticalAxis','y',...
    'VerticalAxisDir','Down')

% Visualize the point cloud
view(player3D,ptCloud);


%% Detect people in left image THIS IS WHERE CHANGES SHOULD BE MADE

peopleDetector = vision.PeopleDetector('MinSize',[166 83]);

% Detect people
bboxes = peopleDetector.step(frameLeftGray)

%% Distance of each person to camera

% Find 3d world coordinates of the centroid of each detected person
% and compute distance from cetroid to camera in meters

centroids = [round(bboxes(:,1)+ bboxes(:,3)/2),...
    round(bboxes(:,2)+bboxes(:,2)/2)];

% Find the 3-D world coordinates of the centroids 
centroidsIdx = sub2ind(size(disparityMap),centroids(:,2),centroids(:,1));
X = points3D(:,:,1);
Y = points3D(:,:,2);
Z = points3D(:,:,3);
centroids3D = [X(centroidsIdx)';Y(centroidsIdx)';Z(centroidsIdx)'];

% Find distance from camera in meters 
dists = sqrt(sum(centroids3D.^2));

% Display the detected people and their distances
labels = cell(1,numel(dists));
for i = 1:numel(dists)
    labels{i} = sprintf('%0.2f meters',dists(i));
end

figure;
imshow(insertObjectAnnotation(frameLeftRect,'rectangle',bboxes,labels));
title('Detected people')

%% Process the rest of the video

while hasFrame(readerLeft) && hasFrame(readerRight)
% read the frames
frameLeft = readFrame(readerLeft);
frameRight = readFrame(readerRight);

%Rectify the frames
[frameLeftRect,frameRightRect] = ...
    rectifyStereoImages(frameLeft,frameRight,stereoParams);

%convert to greyscale
frameLeftGray = rgb2gray(frameLeftRect);

%Compute to disparity
disparityMap = disparitySGM(frameLeftGray,frameRightGray);

% Reconstruct 3-D scene
points3D = reconstructScene(disparityMap,stereoParams);
points3D = points3D./1000;
ptCloud = pointCloud(points3D,'Color',frameLeftRect);
view(player3D,ptCloud);

%detect people
bboxes = peopleDetector.step(frameLeftGray);

if ~isempty(bboxes)
    %find the centroids of detected people
    % Find the centroids of detected people.
        centroids = [round(bboxes(:, 1) + bboxes(:, 3) / 2), ...
            round(bboxes(:, 2) + bboxes(:, 4) / 2)];
        
        % Find the 3-D world coordinates of the centroids.
        centroidsIdx = sub2ind(size(disparityMap), centroids(:, 2), centroids(:, 1));
        X = points3D(:, :, 1);
        Y = points3D(:, :, 2);
        Z = points3D(:, :, 3);
        centroids3D = [X(centroidsIdx), Y(centroidsIdx), Z(centroidsIdx)];
        
        % Find the distances from the camera in meters.
        dists = sqrt(sum(centroids3D .^ 2, 2));
        
        % Display the detect people and their distances.
        labels = cell(1, numel(dists));
        for i = 1:numel(dists)
            labels{i} = sprintf('%0.2f meters', dists(i));
        end
        dispFrame = insertObjectAnnotation(frameLeftRect, 'rectangle', bboxes,...
            labels);
    else
        dispFrame = frameLeftRect;
    end
 
    % Display the frame.
    step(player, dispFrame);
end
    





















