function T = trackingAccuracy(tracked_values,actual_displacement,robot_values)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

Process_Names = ["Tracking Accuracy"; "Robotic Control Sys Accuracy";"End-to-End Accuracy"];
measured_displacement = zeros(3, 1);
accuracy = zeros(3, 1);

tracked_values(tracked_values == 0) = NaN; %Setting Skipped Frames 0 Output value to NaN
tracked_displacement = abs(mean(tracked_values(6:50),'omitnan') - mean(tracked_values(200:231),'omitnan')); %How much tracker Moved
track_accuracy = 100 - abs((tracked_displacement-actual_displacement)./actual_displacement)*100; %Error Percentage

robot_values(robot_values == 0) = NaN; %Setting Skipped Frames 0 Output value to NaN
end_displacement = abs(mean(robot_values(1:50),'omitnan') - mean(robot_values(200:231),'omitnan')); %How much tracker Moved

robot_displacement = abs(mean(robot_values(1:50),'omitnan') - mean(robot_values(200:231),'omitnan')); %How much tracker Moved
robot_accuracy = 100 - abs((robot_displacement-tracked_displacement)./tracked_displacement)*100; %Error Percentage

end_accuracy = 100 - abs((robot_displacement-actual_displacement)./actual_displacement)*100; %Error Percentage

measured_displacement(1,1) = tracked_displacement;
measured_displacement(2,1) = robot_displacement;
measured_displacement(3,1) = end_displacement;

accuracy(1,1) = track_accuracy;
accuracy(2,1) = robot_accuracy;
accuracy(3,1) = end_accuracy;

T = table(Process_Names, measured_displacement, accuracy);

end

