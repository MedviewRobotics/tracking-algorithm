%WEIGHTEDAVERAGE normalizes current location of the surgical tip with
%information from previous frames.
%
%   [surgicalTip_3D_norm] = weightedAverage(surgicalTip_3D, k) returns
%   current normalized location of the surgical tip using raw data from the
%   current frame and the last ten frames. Function first removes outlier
%   values and then applies weighted averaging, giving the most weight to
%   the most recent values.
%    
%   INPUTS:
%       surgicalTip_3D = raw location of the current and past locations of
%           the surgical tip,3xk matrix
%       k = current frame
%
%   OUTPUTS:
%       surgicalTip_3D_norm = new current 3D location of surgical tip, 1x3
%           matrix

function [surgicalTip_3D_norm] = weightedAverage(surgicalTip_3D, k)

%Define weights
W10 = [0.05 0.05  0.05 0.05 0.10 0.10 0.10 0.15 0.15 0.20];
W9 = [0.05  0.05 0.05 0.10 0.10 0.10 0.15 0.20 0.20];
W8 = [0.05 0.05 0.10 0.10 0.10 0.15 0.20 0.25];
W7 = [0.05 0.05 0.10 0.15 0.20 0.20 0.25];
W6 = [0.05 0.10 0.15 0.20 0.25 0.25];
W5 = [0.1  0.15  0.2  0.25  0.3];
W4 = [0.15  0.25  0.25  0.35];

%Create array of recent values
recent_values = [surgicalTip_3D(:, k-9) surgicalTip_3D(:, k-8) surgicalTip_3D(:, k-7) surgicalTip_3D(:, k-6) surgicalTip_3D(:, k-5)  surgicalTip_3D(:, k-4)  surgicalTip_3D(:, k-3)  surgicalTip_3D(:, k-2)  surgicalTip_3D(:, k-1)  surgicalTip_3D(:, k)];

%Remove outliers
remove_outliers_x = rmoutliers(recent_values(1, :), 'mean');
remove_outliers_y = rmoutliers(recent_values(2, :), 'mean');
remove_outliers_z = rmoutliers(recent_values(3, :), 'mean');

%Apply weights X
if size(remove_outliers_x) == [1 10]
    temp_x = remove_outliers_x.*W10;
elseif size(remove_outliers_x) == [1 9]
    temp_x = remove_outliers_x.*W9;
elseif size(remove_outliers_x) == [1 8]
    temp_x = remove_outliers_x.*W8;
elseif size(remove_outliers_x) == [1 7]
    temp_x = remove_outliers_x.*W7;
elseif size(remove_outliers_x) == [1 6]
    temp_x = remove_outliers_x.*W6;
elseif size(remove_outliers_x) == [1 5]
    temp_x = remove_outliers_x.*W5;
elseif size(remove_outliers_x) == [1 4]
    temp_x = remove_outliers_x.*W4;
end

%Apply weights Y
if size(remove_outliers_y) == [1 10]
    temp_y = remove_outliers_y.*W10;
elseif size(remove_outliers_y) == [1 9]
    temp_y = remove_outliers_y.*W9;
elseif size(remove_outliers_y) == [1 8]
    temp_y = remove_outliers_y.*W8;
elseif size(remove_outliers_y) == [1 7]
    temp_y = remove_outliers_y.*W7;
elseif size(remove_outliers_y) == [1 6]
    temp_y = remove_outliers_y.*W6;
elseif size(remove_outliers_y) == [1 5]
    temp_y = remove_outliers_y.*W5;
elseif size(remove_outliers_y) == [1 4]
    temp_y = remove_outliers_y.*W4;
end

%Apply weights Z
if size(remove_outliers_z) == [1 10]
    temp_z = remove_outliers_z.*W10;
elseif size(remove_outliers_z) == [1 9]
    temp_z = remove_outliers_z.*W9;
elseif size(remove_outliers_z) == [1 8]
    temp_z = remove_outliers_z.*W8;
elseif size(remove_outliers_z) == [1 7]
    temp_z = remove_outliers_z.*W7;
elseif size(remove_outliers_z) == [1 6]
    temp_z = remove_outliers_z.*W6;
elseif size(remove_outliers_z) == [1 5]
    temp_z = remove_outliers_z.*W5;
elseif size(remove_outliers_z) == [1 4]
    temp_z = remove_outliers_z.*W4;
end

surgicalTip_3D_norm(1) = sum(temp_x);
surgicalTip_3D_norm(2) = sum(temp_y);
surgicalTip_3D_norm(3) = sum(temp_z);

end

