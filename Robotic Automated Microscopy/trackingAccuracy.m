%TRACKINGACCURACY computes metrics on the overall system accuracy
%
%   T = trackingAccuracy(tracked_values,actual_displacement,robot_values)
%   computes metrics on the overall system accuracy of the automated
%   robotic microscopy system.
%
%   INPUTS:
%       tracked_values = 1xnFramesLeft array that contains the surgical tip
%           coordinates computed by the tracking system over all frames
%       actual_displacement = GT value indicating the movement in the
%           specified direction in the video
%       robot_values = 1xnFramesLeft array that documents the movement of
%           the robot across all frames in response to tracked_values inputs
%
%   OUTPUTS:
%       T = tabulated accuracy of the overall system, the tracking system,
%           and the control system

function T = trackingAccuracy(tracked_values,actual_displacement,robot_values)

Process_Names = ["Tracking Accuracy"; "Robotic Control Sys Accuracy";"End-to-End Accuracy"];
measured_displacement = zeros(3, 1);
accuracy = zeros(3, 1);

tracked_values(tracked_values == 0) = NaN; %Setting Skipped Frames 0 Output value to NaN
tracked_displacement = abs(mean(tracked_values(1:5),'omitnan') - mean(tracked_values(220:229),'omitnan')); %How much tracker moved
track_accuracy = 100 - abs((tracked_displacement-actual_displacement)./actual_displacement)*100; %Error Percentage

robot_values(robot_values == 0) = NaN; %Setting Skipped Frames 0 Output value to NaN
end_displacement = abs(mean(robot_values(1:5),'omitnan') - mean(robot_values(220:230),'omitnan')); %How much tracker Moved

robot_displacement = abs(mean(robot_values(1:50),'omitnan') - mean(robot_values(200:229),'omitnan')); %How much tracker Moved
robot_accuracy = 100 - abs((robot_displacement-tracked_displacement)./tracked_displacement)*100; %Error Percentage

end_accuracy = (robot_accuracy*track_accuracy)/100; %Error Percentage

measured_displacement(1,1) = tracked_displacement;
measured_displacement(2,1) = robot_displacement;
measured_displacement(3,1) = NaN;

accuracy(1,1) = track_accuracy;
accuracy(2,1) = robot_accuracy;
accuracy(3,1) = end_accuracy;

T = table(Process_Names, measured_displacement, accuracy);

end

