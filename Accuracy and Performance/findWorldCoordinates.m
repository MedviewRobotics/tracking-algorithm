%FINDWORLDCOORDINATES determines location of markers in world coordinates
%
%   [point3d_1,point3d_2, point3d_3] = findWorldCoordinates(centroidLeft,centroidRight, stereoParams)
%   returns the position of each active marker in world coordinates,
%   wherein the world coordinate origin is the midpoint between the two
%   cameras. This function uses the embedded triangulate function in MATLAB
%   to compute the 3D position.
%
%   INPUTS:
%       centroidLeft = location of all three markers in pixel coordinates
%           from the perspective of the left camera, stored in 3x2 matrix.
%       centroidRight = location of all three markers in pixel coordinates
%           from the perspective of the right camera, stored in 3x2 matrix.
%       stereoParams = intrinsic and extrinsic parameters computed for the
%           set of cameras used for acquisition, 1x1 stereoParameters
%           object generated from calibration images using embedded Matlab
%           functions.
%
%   OUTPUTS:
%       point3d_1 = location of the green active marker in world
%           coordinates, stored in 1x3 array
%       point3d_2 = location of the blue active marker in world
%           coordinates, stored in 1x3 array
%       point3d_3 = location of the red active marker in world
%           coordinates, stored in 1x3 array

function [point3d_1,point3d_2, point3d_3] = findWorldCoordinates(centroidLeft,centroidRight, stereoParams)

point3d_1 = triangulate(centroidLeft(1,:),centroidRight(1,:),stereoParams);
point3d_2 = triangulate(centroidLeft(2,:),centroidRight(2,:),stereoParams);
point3d_3 = triangulate(centroidLeft(3,:),centroidRight(3,:),stereoParams);

end

