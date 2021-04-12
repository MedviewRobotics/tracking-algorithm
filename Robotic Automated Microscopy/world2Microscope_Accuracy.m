%WORLD2MICROSCOPE_ACCURACY converts world coordinates into local robot
%coordinates
%
%   [x_output, y_output, z_output] = world2Microscope_Accuracy(x_input, y_input, z_input,x_origin,y_origin,z_origin)
%   takes the location of the surgical tip output from the tracking system
%   and translates them into the local coordinates of the robot.
%
%   INPUTS:
%       x_input = x coordinate of the surgical instrument in world
%           coordinates
%       y_input = y coordinate of the surgical instrument in world
%           coordinates
%       y_input = y coordinate of the surgical instrument in world
%           coordinates
%       x_origin = x coordinate of the robot origin in world coordinates
%       y_origin = y coordinate of the robot origin in world coordinates
%       z_origin = z coordinate of the robot origin in world coordinates
%
%   OUTPUTS:
%       x_output = x coordinate of the surgical instrument in local
%           coordinates
%       y_output = y coordinate of the surgical instrument in local
%           coordinates
%       z_output = z coordinate of the surgical instrument in local
%           coordinates

function [xMicroscope, yMicroscope, zMicroscope] = world2Microscope_Accuracy(x_input, y_input, z_input,x_origin,y_origin,z_origin)

lclCoord = global2localcoord([x_input;-y_input;z_input],'rr',[x_origin;y_origin; z_origin]);

%Calculate tip locations
yMicroscope = lclCoord(1); %X from Tracking
xMicroscope = lclCoord(3); %Z from Tracking Output
zMicroscope = lclCoord(2); %Y from Trakcing

end

