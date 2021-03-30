%MOVEMICROSCOPE initiates control system to move robot
%
%   [newq0,X,Y,Z,Q] = moveMicroscope(input_x,input_y,input_z,q0,AT03)
%   initiates movement of the robot in response to new surgical tip
%   coordinates from the tracking system.
%
%   INPUTS:
%       input_x = new X coordinates of the surgical tip in local
%           coordinates, stored in 1x3 array.
%       input_y = new Y coordinates of the surgical tip in local
%           coordinates, stored in 1x3 array.
%       input_z = new Z coordinates of the surgical tip in local
%           coordinates, stored in 1x3 array.
%       q0 = previous robot location, stored in 1x6 array
%       AT03 = robot unique identifier
%
%   OUPUTS:
%       newq0 = output robot location, required as an input in the
%           following initiation
%       X = ?
%       Y = ?
%       Z = ?

function [newq0,X,Y,Z,Q] = moveMicroscope(input_x,input_y,input_z,q0,AT03)

%while i<20

R = 3.1416;
P = 0;
Y = 0;

%Call safety protocol function
[x, y, z] = safetyprotocols(input_x, input_y, input_z); 

% Get q_new to move to new pose
T_start = transl(x,y,z) * rpy2tr(R,P,Y);
q0;
qf = AT03.ikine(T_start,'q0',q0);

%move to new pose
[Q,QD,QDD] = jtraj(q0, qf, 10);
%AT03.plot(Q);
%AT03.plot3d(Q,'view','y','path','C:\Users\bluet\Documents\Capstone_New\OTS\tracking-algorithm\Accuracy and Performance\robot\data\ARTE4','floorlevel',-175,'base');

%New pose is now the old/start pose
q0 = Q(end,:);
T_start = AT03.fkine(q0);
[X,Y,Z] = transl(T_start);

newq0 = q0;

end
