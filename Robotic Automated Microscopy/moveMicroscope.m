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
%       eul = found from the rotation matrix using rotm2eul()
%
%   OUTPUTS:
%       newq0 = output robot location, required as an input in the
%           following initiation
%       X,Y,Z = The end-effector end position calculated by fkine 
%       R,P,Y = Roll, Pitch, Yaw values from rotation matrix to Euler.
%       Q = joint space trajectory Q (MxN) where the joint coordinates vary from Q0 (1xN) to QF (1xN)

function [newq0,x,y,z,R,P,Y,Q] = moveMicroscope(input_x,input_y,input_z,q0,AT03, eul)

%Call safety protocol function
[x, y, z, R, P, Y] = safetyprotocols(input_x, input_y, input_z, eul); 

% Get q_new to move to new pose
T_start = transl(x,y,z) * rpy2tr(R,P,Y);
qf = AT03.ikine(T_start,'q0',q0);

%move to new pose
[Q,QD,QDD] = jtraj(q0, qf, 10);

%New pose is now the old/start pose
q0 = Q(end,:);
T_start = AT03.fkine(q0);
[x,y,z] = transl(T_start);
newq0 = q0;

end
