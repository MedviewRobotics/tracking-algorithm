function [bw2,centroids,diffFrameColor] = detectmarkerColorDark(img,threshold,RGB_channel,radius)

diffFrameColor= imsubtract(img(:,:,RGB_channel), rgb2gray(img)); % Get color component of the image
[M,N] = size(diffFrameColor);
figure;imshow(diffFrameColor)
diffFrameColor(1:M,[1:0.3*N 0.65*N:N],:)=0;
diffFrameColor([1:200 550:M],1:N,:)=0;
figure
imshow(diffFrameColor)
%diffFrameColor = medfilt2(diffFrameColor, [3 3]); % Filter out the noise by using median filter
binFrameColor = imbinarize(diffFrameColor, threshold); % Convert the image into binary image with the blue objects as white
% figure;
% imshowpair(img,binFrameColor,'montage')
% title('Binarized Image')
%radius_red = 1;
%radius_green = 10;
%radius_blue = 0;

image= imdilate(binFrameColor, strel('disk', radius));
bw2 = imfill(image,'holes');
% figure
% imshow(bw2)
% title('Post processed image')


s = regionprops(binFrameColor,'centroid');
centroids = cat(1,s.Centroid);
% r1 = centroids(1,1);
% c1 = centroids(1,2);
% contour = bwtraceboundary(binFrameColor,round([r1 c1]),'W');
% figure
% imshow(img)
% hold on
% plot(contour(:,2),contour(:,1),'g','LineWidth',2)
% hold off


end





