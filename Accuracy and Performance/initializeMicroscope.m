%INITIALIZEMICROSCOPE initializes the robot simulation model
%
%   [AT03,q0] = initializeMicroscope() initializes the robot simulation 
%   model and computes the starting pose of the robot.
%
%   OUTPUTS:
%       AT03 = name of robot entity
%       q0 = starting pose of the robot

function [AT03,q0] = initializeMicroscope()

% Initialize Robot SerialLink
L(1)=Link ([0 28 0 pi/2]);
L(2)=Link ([0 0 126.5 0]);
L(3)=Link ([0 0 0 pi/2]);
L(4)=Link ([0 93.95 0 -pi/2]);
L(5)=Link ([0 0 0 pi/2]);
L(6)=Link ([0 5.5 0 0]);

AT03=SerialLink(L,'name','AT03Robot');
start_pos_new = [0 3.14/2 0 0 -3.14/2 0];
T_start = AT03.fkine(start_pos_new);
[Start_X,Start_Y,Start_Z] = transl(T_start);
AT03.plot(start_pos_new);

q0=start_pos_new;

end

