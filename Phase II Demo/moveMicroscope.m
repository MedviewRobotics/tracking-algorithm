function [] = moveMicroscope(inputArg1,inputArg2) %% change input arguments

while i<20
    %AT03.plot(q0)
    %T_start = AT03.fkine(q0) 
    x = rx(i); %new pose stream in
    y = ry(i); %new pose _ stream in
    z = rz(i); %new pose _ stream in
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
    i = i + 1;
    %loop
end

% Troubleshooting plot3d
% stltest0 = 'C:\Users\bluet\Documents\Capstone_New\Control_System_New\robot\data\meshes\CAPSTONE\AT03\link0.stl';
% stltest1 = 'C:\Users\bluet\Documents\Capstone_New\Control_System_New\robot\data\meshes\CAPSTONE\AT03\link1.stl';
% stltest2 = 'C:\Users\bluet\Documents\Capstone_New\Control_System_New\robot\data\meshes\CAPSTONE\AT03\link2.stl';
% stltest3 = 'C:\Users\bluet\Documents\Capstone_New\Control_System_New\robot\data\meshes\CAPSTONE\AT03\link3.stl';
% stltest4 = 'C:\Users\bluet\Documents\Capstone_New\Control_System_New\robot\data\meshes\CAPSTONE\AT03\link4.stl';
% stltest5 = 'C:\Users\bluet\Documents\Capstone_New\Control_System_New\robot\data\meshes\CAPSTONE\AT03\link5.stl';
% stltest6 = 'C:\Users\bluet\Documents\Capstone_New\Control_System_New\robot\data\meshes\CAPSTONE\AT03\link6.stl';
% 
% stlread0 = stlread('C:\Users\bluet\Documents\Capstone_New\Control_System_New\robot\data\meshes\CAPSTONE\AT03\link0.stl');
% stlread1 = stlread('C:\Users\bluet\Documents\Capstone_New\Control_System_New\robot\data\meshes\CAPSTONE\AT03\link1.stl');
% stlread2 = stlread('C:\Users\bluet\Documents\Capstone_New\Control_System_New\robot\data\meshes\CAPSTONE\AT03\link2.stl');
% stlread3 = stlread('C:\Users\bluet\Documents\Capstone_New\Control_System_New\robot\data\meshes\CAPSTONE\AT03\link3.stl');
% stlread4 = stlread('C:\Users\bluet\Documents\Capstone_New\Control_System_New\robot\data\meshes\CAPSTONE\AT03\link4.stl');
% stlread5 = stlread('C:\Users\bluet\Documents\Capstone_New\Control_System_New\robot\data\meshes\CAPSTONE\AT03\link5.stl');
% stlread6 = stlread('C:\Users\bluet\Documents\Capstone_New\Control_System_New\robot\data\meshes\CAPSTONE\AT03\link6.stl');

model0 = createpde(3);
importGeometry(model0,stltest0);
figure
pdegplot(model0,'FaceLabels','on')
trimesh(stlread0); 

model1 = createpde(3);
importGeometry(model1,stltest1);
figure
pdegplot(model1,'FaceLabels','on')

model2 = createpde(3);
importGeometry(model2,stltest2);
figure
pdegplot(model2,'FaceLabels','on')

model3 = createpde(3);
importGeometry(model3,stltest3);
figure
pdegplot(model3,'FaceLabels','on')

model4 = createpde(1);
importGeometry(model4,stltest4);
figure
pdegplot(model4,'FaceLabels','on')

model5 = createpde(3);
importGeometry(model5,stltest5);
figure
pdegplot(model5,'FaceLabels','on')
end

