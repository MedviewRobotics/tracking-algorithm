function [x_output, y_output, z_output] = world2Microscope_Accuracy(x_input, y_input, z_input)

x_world_to_local = -400;
y_world_to_local = 30;
z_world_to_local = 1850;
lclCoord = global2localcoord([x_input;y_input;z_input],'rr',[x_world_to_local;...
    y_world_to_local; z_world_to_local]);

%%
%y_output = -0.1389*lclCoord(1); %X from Tracking Output
y_output = lclCoord(1); %X from Tracking

%%

x_output = lclCoord(3); %Z from Tracking Output

% x_output_temp = -0.233*lclCoord(3); %Z from Tracking Output
% if x_output_temp<20
%     x_output = 20;
% elseif x_output_temp>70
%     x_output = 70;
% else
%     x_output = x_output_temp;
% end

%%
%z_output = 0.075*lclCoord(2); %Y from Trakcing Output
z_output = lclCoord(2); %Y from Trakcing

end

