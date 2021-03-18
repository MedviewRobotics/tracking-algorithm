function [AT03,q0] = initializeMicroscope()
%x_frame1,y_frame1,z_frame1
% % Initialize Toolbox (Robotics Toolbox for MATLAB (release 10.4))
% startup_rvc;
% toolboxFile = 'RTB (1).mltbx';
% installedToolbox = matlab.addons.toolbox.installToolbox(toolboxFile);

% Initialize Robot SerialLink
L(1)=Link ([0 28 0 pi/2]);
L(2)=Link ([0 0 126.5 0]);
L(3)=Link ([0 0 0 pi/2]);
L(4)=Link ([0 93.95 0 -pi/2]);
L(5)=Link ([0 0 0 pi/2]);
L(6)=Link ([0 5.5 0 0]);

AT03=SerialLink(L,'name','AT03Robot');
%test ... AT03.model3d = 'CAPSTONE\AT03';
%start_pos=[0,1.571/2,1.571,-1.571,-1.571/2,0];
start_pos_new = [0 3.14/2 0 0 -3.14/2 0];
%AT03.offset=[0 0 0 0 0 0];
T_start = AT03.fkine(start_pos_new);
[Start_X,Start_Y,Start_Z] = transl(T_start);
AT03.plot(start_pos_new);

%x_frame1 = -36; y_frame1= 17; z_frame1=100;
% Alt_Start_X = x_frame1 ; % -100;
% Alt_Start_Y = y_frame1; %0;
% Alt_Start_Z = z_frame1; %112;
% R = 3.1416; %new pose orientation input
% P = 0; %new pose orientation input
% Y = 0; %new pose orientation input
% T_alt_start = transl(Alt_Start_X,Alt_Start_Y,Alt_Start_Z )*rpy2tr(R,P,Y);
% q0_alt = AT03.ikine(T_alt_start);%'q0',start_pos_new);
% AT03.plot(q0_alt);
% 
% q0 = q0_alt;
q0=start_pos_new;

% %test
% AT03=SerialLink(L,'name','AT03Robot');
% %AT03.model3d = 'CAPSTONE\AT03';
% %start_pos=[0,1.571/2,1.571,-1.571,-1.571/2,0];
% start_pos_new = [0 3.14/2 0 0 -3.14/2 0];
% %AT03.offset=[0 0 0 0 0 0];
% T_start = AT03.fkine(start_pos_new);
% RPY = tr2rpy(T_start);
% [X,Y,Z] = transl(T_start);
% 
% x = 125; %new pose stream in
% y = -71; %new pose _ stream in
% z = 65; %new pose _ stream in
% R = 3.1416;
% P = 0;
% Y = 0;
% % Get q_new to move to new pose
% T_start = transl(x,y,z) * rpy2tr(R,P,Y);
% q0 = start_pos_new;
% qf = AT03.ikine(T_start,'q0',q0);
 
% % Initialize 
% % start_pos_new = [0 3.14/2 0 0 -3.14/2 0]
% q0 = qf; %first q0 to give starting pose.

end

