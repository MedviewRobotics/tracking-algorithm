function [x_output, y_output, z_output] = world2Microscope(x_input, y_input, z_input)

lclCoord = global2localcoord([x_input;y_input;z_input],'rr',[-122; -1482; 1600]);%might have to shift y up?

% Y IN JAY
% x_min %-410 %TRY -50 currently mid -122
% x_max %166  %TRY 50
y_output = -0.1389*lclCoord(1); %Z from Tracking

% correct outliers and shift all values found from camera depth to patient
% bed values

x_output_temp = -0.233*lclCoord(3); %X from Tracking
if x_output_temp<20
    x_output = 20;
elseif x_output_temp>70
    x_output = 70;
else
    x_output = x_output_temp;
end

%
z_output = 0.075*lclCoord(2); %Y from Trakcing

end

