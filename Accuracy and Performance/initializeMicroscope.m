%INITIALIZEMICROSCOPE initializes the robot simulation model
%
%   [AT03,q0] = initializeMicroscope() initializes the robot simulation 
%   model and computes the starting pose of the robot.
%
%   OUTPUTS:
%       AT03 = name of robot entity
%       q0 = starting pose of the robot

function [AT03,q0] = initializeMicroscope(x_origin,y_origin,z_origin,eul)

% Initialize Robot SerialLink
L(1)=Link ([0 28 0 pi/2]);
L(2)=Link ([0 0 126.5 0]);
L(3)=Link ([0 0 0 pi/2]);
L(4)=Link ([0 93.95 0 -pi/2]);
L(5)=Link ([0 0 0 pi/2]);
L(6)=Link ([0 5.5 0 0]);

AT03=SerialLink(L,'name','AT03Robot');
%start_pos_new = [0 3.14/2 0 0 -3.14/2 0];
q0 = [0 3.14/2 0 -3.14 -3.3/2 0];
%T_start = AT03.fkine(start_pos_new);
%[Start_X,Start_Y,Start_Z] = transl(T_start);

% R = eul(3) %3 
% %R = cast(eul(2),'uint8');
% P = eul(1) %1
% %P = cast(eul(3),'uint8');
% Y = eul(2) %2
% %Y = cast(eul(1),'uint8');

input_y = 0;

input_x = 90;

input_z = 159;
%Call safety protocol function
[x, y, z, R, P, Y] = safetyprotocols(input_x, input_y, input_z,eul);

% Get q_new to move to new pose
T_start = transl(x,y,z) * rpy2tr(R,P,Y);

start_pos_new = AT03.ikine(T_start,'q0',q0);
q0=start_pos_new;
AT03.plot(q0);

end

