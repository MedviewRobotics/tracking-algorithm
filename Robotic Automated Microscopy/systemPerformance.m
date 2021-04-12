%SYSTEMPERFORMANCE computes metrics on the overall system performance
%
%   [T, Equiv_FPS_Rate] = systemPerformance(elapsed_1,elapsed_2, elapsed_3, elapsed_4)
%   computes metrics on the overall system performance of the automated
%   robotic microscopy system.
%
%   INPUTS:
%       elapsed_1 = 1xnFramesLeft array that stored the computational time
%           elapsed for the preprocessing code in every frame
%       elapsed_2 = 1xnFramesLeft array that stored the computational time
%           elapsed for the finding surgical tip code in every frame
%       elapsed_3 = 1xnFramesLeft array that stored the computational time
%           elapsed for the world2Microscope code in every frame
%       elapsed_4 = 1xnFramesLeft array that stored the computational time
%           elapsed for the control system in every frame
%
%   OUTPUTS:
%       T = table showing performance metrics for "Entire Worflow" 
%           "Preprocessing", "Find Tip", "World2Microscope" and "Control 
%           System" phases
%       Equiv_FPS_Rate = computes the equivalent rate of the entire system
%           in terms of frames per second

function [T, Equiv_FPS_Rate] = systemPerformance(elapsed_1,elapsed_2, elapsed_3, elapsed_4)

elapsedcut_1 = elapsed_1(5:235);
elapsedcut_2 = elapsed_2(5:235);
elapsedcut_3 = elapsed_3(5:235);
elapsedcut_4 = elapsed_4(5:235);

Process_Names = ["Entire Worflow"; "Preprocessing";"Find Tip"; "World2Microscope"; "Control System"];
Avg_Time = zeros(5, 1);
Stdv_Time = zeros(5, 1);
Max_Time = zeros(5, 1);
Min_Time = zeros(5, 1);

for i = 1:4
    array = strcat('elapsedcut_', num2str(i));
    Avg_Time(i+1, 1) = mean(eval(array));
    Stdv_Time(i+1, 1) = std(eval(array));
    Max_Time(i+1, 1) = max(eval(array));
    Min_Time(i+1, 1) = min(eval(array));
end

Avg_Time(1, 1) = sum(Avg_Time(2:5,:));
Equiv_FPS_Rate = 1/(Avg_Time(1, 1));
Stdv_Time(1, 1) = sqrt(sum((Stdv_Time(2:5,:)).^2));
Max_Time(1, 1) = sum(Max_Time(2:5,:));
Min_Time(1, 1) = sum(Min_Time(2:5,:));

T = table(Process_Names, Avg_Time, Stdv_Time, Max_Time, Min_Time);

end

