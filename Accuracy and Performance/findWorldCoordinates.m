function [point3d_1,point3d_2, point3d_3] = findWorldCoordinates(centroidLeft,centroidRight, stereoParams)

point3d_1 = triangulate(centroidLeft(1,:),centroidRight(1,:),stereoParams);
point3d_2 = triangulate(centroidLeft(2,:),centroidRight(2,:),stereoParams);
point3d_3 = triangulate(centroidLeft(3,:),centroidRight(3,:),stereoParams);

end

