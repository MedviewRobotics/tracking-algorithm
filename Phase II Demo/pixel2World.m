function [x_output, y_output, z_output, z_out_string] = pixel2World(x_input, y_input, z_input)

%Z coordinate
z_output = norm(z_input);
z_out_string = sprintf('%d',round(z_output));

%X coordinate
x_output = x_input/10; %will change

%Y coordinate
y_output = y_input/10; %will change

end

