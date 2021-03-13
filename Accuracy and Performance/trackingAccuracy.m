function [tracked_displacement,accuracy] = trackingAccuracy(tracked_values,actual_displacement)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

tracked_values(tracked_values == 0) = NaN; %Setting Skipped Frames 0 Output value to NaN
tracked_displacement = abs(min(tracked_values(1:50)) - max(tracked_values(200:231))); %How much tracker Moved
accuracy = 100 - abs((tracked_displacement-actual_displacement)./actual_displacement)*100; %Error Percentage

end

