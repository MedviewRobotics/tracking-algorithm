function newq0 = moveMicroscope(input_x,input_y,input_z,q0) %% change input arguments

%while i<20
x = input_x; %new pose stream in
y = input_y; %new pose _ stream in
z = input_z; %new pose _ stream in
R = 3.1416;
P = 0;
Y = 0;
% Get q_new to move to new pose
T_start = transl(x,y,z) * rpy2tr(R,P,Y);
qf = AT03.ikine(T_start,'q0',q0);

%move to new pose
[Q,QD,QDD] = jtraj(q0, qf, 15);
AT03.plot(Q);

%AT03.plot3d(Q,'joints','view','y','path','C:\Users\bluet\Documents\Capstone_New\Control_System_New\robot\data\meshes\CAPSTONE\AT03','floorlevel',0)

%new pose is now the old/start pose
q0 = Q(end,:);
T_start = AT03.fkine(q0);
[X,Y,Z] = transl(T_start);

newq0 = q0;

%i = i + 1;
%loop
%end

end

