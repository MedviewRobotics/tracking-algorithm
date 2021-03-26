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

function [xMicroscope, yMicroscope, zMicroscope] = safetyProtocols(xMicroscope_In, yMicroscope_In, zMicroscope_In)

%Z from Tracking Output
if xMicroscope_In<35
     xMicroscope = 35;
elseif xMicroscope_In>115
     xMicroscope = 115;
else
     xMicroscope = xMicroscope_In;
end

%X from Tracking Output
if yMicroscope_In<-70
     yMicroscope = -70;
elseif yMicroscope_In>70
     yMicroscope = 70;
else
     yMicroscope = yMicroscope_In;
end

%Y from Tracking Output
if zMicroscope_In<100
     zMicroscope = 100;
elseif zMicroscope_In>170
     zMicroscope = 170;
else
     zMicroscope = zMicroscope_In;
end

end

