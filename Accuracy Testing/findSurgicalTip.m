function [surgicalTip_3D] = findSurgicalTip(green_3D,blue_3D,red_3D, pivotOffset)

%Determine point X between blue and green marker (37.5 mm away from green)
g2b = blue_3D - green_3D;
g2b_norm = g2b/norm(g2b);
midpoint = green_3D + g2b_norm*37.5;

%Find direction vector from midpoint to red marker
m2r = red_3D - midpoint;
m2r_norm = m2r/norm(m2r);

%Find location of surgical tip
surgicalTip_3D = midpoint + m2r_norm*pivotOffset;

end