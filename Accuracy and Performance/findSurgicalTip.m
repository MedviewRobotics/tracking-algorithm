%FINDSURGICALTIP determines the location and orientation of the surgical
%instrument.
%
%   [surgicalTip_3D, rotMatrix] = FINDSURGICALTIP(green_3D,blue_3D,red_3D, pivotOffset)
%   returns the position of the surgical tip and the orientation of the 
%   surgical instrument 3 x 3 Matrix
%
%   INPUTS:
%       green_3D = 3D location of the green marker in world coordinates, 3x1 matrix
%       blue_3D = 3D location of the blue marker in world coordinates, 3x1 matrix
%       red_3D = 3D location of the red marker in world coordinates, 3x1 matrix
%       pivotOffset = distance from the midpoint of the blue and green
%           markers to the tip of the instrument, double value
%
%   OUTPUTS:
%       surgicalTip_3D = 3D position of the surgical tip in world
%           coordinates
%       rotMatrix = rotation matrix describing the orientation of the
%           surgical tip, 3x3 matrix


function [surgicalTip_3D, rotMatrix] = findSurgicalTip(green_3D,blue_3D,red_3D, pivotOffset)

%Determine point X between blue and green marker (37.5 mm away from green)
g2b = blue_3D - green_3D;
g2b_norm = g2b/norm(g2b);
midpoint = green_3D + g2b_norm*37.5;

%Find direction vector from midpoint to red marker
m2r = red_3D - midpoint;
m2r_norm = m2r/norm(m2r);

%Find location of surgical tip
surgicalTip_3D = midpoint + m2r_norm*pivotOffset;

%Find rotation matrix
A=orth([green_3D(:)-blue_3D(:),green_3D(:)-red_3D(:)]);
B=[1 0 0; 0 1 0].';
reg=absor(A,B,'doTrans',0);
rotMatrix=reg.R;

end