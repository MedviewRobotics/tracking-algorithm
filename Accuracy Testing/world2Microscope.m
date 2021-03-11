function [x_output, y_output, z_output] = world2Microscope(x_input, y_input, z_input)

x_world_to_local = -122;
y_world_to_local = -1482;
z_world_to_local = 1600;
lclCoord = global2localcoord([x_input;y_input;z_input],'rr',[x_world_to_local;...
    y_world_to_local; z_world_to_local]);

%%
y_output = -0.1389*lclCoord(1); %Z from Tracking

%%
x_output_temp = -0.233*lclCoord(3); %X from Tracking
if x_output_temp<20
    x_output = 20;
elseif x_output_temp>70
    x_output = 70;
else
    x_output = x_output_temp;
end

%%
z_output = 0.075*lclCoord(2); %Y from Trakcing

end

