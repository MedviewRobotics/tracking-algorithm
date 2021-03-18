function [x_output, y_output, z_output] = world2Microscope(x_input, y_input, z_input)

x_world_to_local = -158;
y_world_to_local = -205;
z_world_to_local = 1450;
lclCoord = global2localcoord([x_input;y_input;z_input],'rr',[x_world_to_local;...
    y_world_to_local; z_world_to_local]);

%%
y_output = lclCoord(1); %X from Tracking

x_output = lclCoord(3); %Z from Tracking Output

z_output = lclCoord(2); %Y from Trakcing


end

