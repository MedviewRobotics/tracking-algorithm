function newq0 = moveMicroscope(input_x,input_y,input_z,q0,AT03) %% change input arguments

%while i<20

x = input_x; %new pose stream in
y = input_y;%new pose _ stream in
z = input_z; %new pose _ stream in
R = 3.1416;
P = 0;
Y = 0;

% Get q_new to move to new pose
T_start = transl(x,y,z) * rpy2tr(R,P,Y);
q0;
qf = AT03.ikine(T_start,'q0',q0);

%move to new pose
[Q,QD,QDD] = jtraj(q0, qf, 10);
%AT03.plot(Q);

%New pose is now the old/start pose
q0 = Q(end,:);
T_start = AT03.fkine(q0);
[X,Y,Z] = transl(T_start);

newq0 = q0;

end
