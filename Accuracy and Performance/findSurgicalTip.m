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
%       eul = orientation of surgical instrument in roll, pitch yaw,
%           measured in radians, 3x1 matrix


function [surgicalTip_3D, eul, normal] = findSurgicalTip(green_3D,red_3D,blue_3D, pivotOffset)

%Determine point X between blue and green marker (37.5 mm away from green)
g2b = blue_3D - green_3D;
g2b_norm = g2b/norm(g2b);
midpoint = green_3D + g2b_norm*27.5;

%Find direction vector from midpoint to red marker
m2r = red_3D - midpoint;
m2r_norm = m2r/norm(m2r);

%Find location of surgical tip
surgicalTip_3D = midpoint + m2r_norm*pivotOffset;

%Define two vectors on plane
v1 = green_3D(:)-blue_3D(:);
v2 = green_3D(:)-red_3D(:);

%Find rotation matrix
A=orth([v1,v2]);
B=[1 0 0; 0 1 0].';
reg=absor(A,B,'doTrans',0);
rotMatrix=reg.R;

%Find orientation in roll, pitch, yaw
eul = rotm2eul(rotMatrix);

%Find normal
normal = cross((v1/norm(v1)),(v2/norm(v2)));

end