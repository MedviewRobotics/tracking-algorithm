function [T, Equiv_FPS_Rate] = systemPerformance(elapsed_1,elapsed_2, elapsed_3, elapsed_4)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

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

