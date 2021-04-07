%SAFETYPROTOCOLS implements safety protocols to limit the movement of
%robot.
%
%   [xMicroscope, yMicroscope, zMicroscope] = safetyProtocols(xMicroscope_In, yMicroscope_In, zMicroscope_In)
%   analyzes the position of the surgical instrument being passed to the
%   control system from the tracking system and validates that the new
%   location of the surgical tip is within the means of the robot's range.
%
%   INPUTS:
%       xMicroscope_In = x coordinate of the surgical instrument in local
%           coordinates
%       yMicroscope_In = y coordinate of the surgical instrument in local
%           coordinates
%       zMicroscope_In = z coordinate of the surgical instrument in local
%           coordinates
%
%   OUTPUTS:
%       xMicroscope = x coordinate of the surgical instrument in local
%           coordinates, with adjustments made if the input value did not
%           meet the safety requirements of the robot.
%       yMicroscope = y coordinate of the surgical instrument in local
%           coordinates, with adjustments made if the input value did not
%           meet the safety requirements of the robot.
%       zMicroscope = z coordinate of the surgical instrument in local
%           coordinates, with adjustments made if the input value did not
%           meet the safety requirements of the robot.

function [xMicroscope, yMicroscope, zMicroscope, R_out, P_out, Y_out] = safetyProtocols(xMicroscope_In, yMicroscope_In, zMicroscope_In,eul)

%Roll...About the Z axis (Tracking Y) (Rotate the end effector in XY PLane
R = eul(2);
 if R<-0.6
      R_out = -0.6;
 elseif R>0.6
     R_out = 0.6;
 else
      R_out = R;
 end

%Pitch...Robot X Axis (Tracking Z)
P = eul(3);
 if P<2
      P_out = 2;
 elseif P>3
      P_out = 3;
 else
      P_out = P;
 end

%Yaw..%Robot Yaw in Robot Y axis is from Tracking X
Y = eul(1);
 if Y<-0.6
      Y_out = -0.6;
 elseif Y>0.6
      Y_out = 0.6;
 else
      Y_out = Y;
 end

%Z from Tracking Output 3 --> 1
if xMicroscope_In<35
     xMicroscope = 35;
elseif xMicroscope_In>115
     xMicroscope = 115;
else
     xMicroscope = xMicroscope_In;
end

%X from Tracking Output 1 --> 2
if yMicroscope_In<-70
     yMicroscope = -70;
elseif yMicroscope_In>70
     yMicroscope = 70;
else
     yMicroscope = yMicroscope_In;
end

%Y from Tracking Output 2 --> 3
if zMicroscope_In<90
     zMicroscope = 90;
elseif zMicroscope_In>180
     zMicroscope = 180;
else
     zMicroscope = zMicroscope_In;
end



end