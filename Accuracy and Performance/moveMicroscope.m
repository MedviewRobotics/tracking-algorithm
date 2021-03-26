function [newq0,X,Y,Z,Q] = moveMicroscope(input_x,input_y,input_z,q0,AT03) %% change input arguments

%while i<20
R = 3.1416;
P = 0;
Y = 0;

%Call safety protocol function
[x, y, z] = safetyProtocols(input_x, input_y, input_z); 

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

end

