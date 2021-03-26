function [x_origin,y_origin,z_origin] = find_origin(x_world_test_new,y_world_test_new,z_world_test_new)


x_origin = mean(x_world_test_new(10:end));
y_origin = mean(min(y_world_test_new(10:50),min(y_world_test_new(50:100)))) - 100;
z_world_test_new;
z_origin = 1750; %same distance from Camera every accuracy trial

end

