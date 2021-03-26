function [centroidLeft, bboxLeft, centroidRight, bboxRight] = findCentroids(frameLeftGray,frameRightGray, threshold, hblob)

%Detect markers in the Left 
img_left = frameLeftGray > threshold; %Creates binary image
img_left = bwareaopen(img_left, 22);
img_left = imerode(img_left,strel('disk',1));
img_left = imdilate(img_left,strel('disk',6));

%Detect markers in the Right
img_right = frameRightGray > threshold; %Creates binary image
img_right = bwareaopen(img_right, 22);
img_right = imerode(img_right,strel('disk',1));
img_right = imdilate(img_right,strel('disk',6));

%Find features of the blobs and filter out any that don't meet criteria
cc_left = bwconncomp(img_left);
stats_left = regionprops(cc_left,'Area','Centroid','BoundingBox','Eccentricity','Circularity','Extent','EquivDiameter'); 
idx_left = find([stats_left.Area] > 165 & [stats_left.Area] < 550 & [stats_left.Eccentricity] > 0.1 & [stats_left.Eccentricity] < 0.67);% & ...
BW2_left = ismember(labelmatrix(cc_left),idx_left);  

cc_right = bwconncomp(img_right);
stats_right = regionprops(cc_right,'Area','Centroid','BoundingBox','Eccentricity','Circularity','Extent','EquivDiameter'); 
idx_right = find([stats_right.Area] > 165 & [stats_right.Area] < 550 & [stats_right.Eccentricity] > 0.1 & [stats_right.Eccentricity] < 0.67);% & ...
BW2_right = ismember(labelmatrix(cc_right),idx_right);  

%Find Centroid and Bounding Box of each marker
[centroidLeft,bboxLeft] = step(hblob,BW2_left);
[centroidRight,bboxRight] = step(hblob,BW2_right);

end

