function [x_output, y_output, z_output] = world2Microscope(x_input, y_input, z_input)

lclCoord = global2localcoord([x_input;y_input;z_input],'rr',[0; 0; 320]);%might have to shift y up?

x_output = lclCoord(1);
z_output = (-1)*lclCoord(2);
y_output = lclCoord(3);

end

