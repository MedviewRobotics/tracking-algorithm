function [x_output, y_output, z_output] = world2Microscope(x_input, y_input, z_input)

lclCoord = global2localcoord([x_input;y_input;z_input],'rr',[30; -50; 1650]);%might have to shift y up?

x_output = 0.4*lclCoord(3); %X from Tracking
z_output = 0.5*lclCoord(2); %Y from Trakcing
y_output = 0.2*lclCoord(1); %Z from Tracking

end

