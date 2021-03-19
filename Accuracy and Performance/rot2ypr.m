function [ypr,Q]=rot2ypr(R)
%Map a 3D rotation matrix to yaw-pitch-roll (intrinsic z-y'-x'' rotation)
%Source: https://www.mathworks.com/matlabcentral/answers/437098-given-a-plane-defined-by-three-points-in-space-how-do-i-get-the-roll-and-pitch-angles-euler-angles
%
% [ypr,quat]=rot2ypr(R)
%
%in:
%
% R: A 3D rotation matrix
%
%out:
%
% ypr: A vector of yaw-pitch-roll angles in degrees [yaw,pitch,roll]
% quat: Corresponding unit quaternion [q0 qx qy qz]' with  max(quat)=max(abs(quat))>0


Q=rot2quatern(R);

%%Convert quaternion form to yaw-pitch-roll
[qr,qi,qj,qk]=deal(Q(1),Q(2),Q(3),Q(4));

roll=atan2d( 2*(qr*qi+qj*qk) , 1-2*(qi^2+qj^2) );
pitch=asind( 2*(qr*qj-qk*qi) );
yaw=atan2d( 2*(qr*qk+qi*qj) , 1-2*(qj^2+qk^2) );

ypr=[yaw,pitch,roll];
end

function quat=rot2quatern(R)
%first get result in quaternion form
C= [1 1 1 1  ;...
    1 1 -1 -1 ;...
    1 -1 1 -1;...
    1 -1 -1 1];

d=[1;diag(R)];

qsquares=C*d/4 ;
qsquares(qsquares<0)=0;
qmags=sqrt(qsquares);

[qmax,ii]=max(qmags);
inds=1:4;
inds(ii)=[];

DD=[ R(3,2)-R(2,3);
    R(1,3)-R(3,1);
    R(2,1)-R(1,2);
    R(2,1)+R(1,2);
    R(3,2)+R(2,3);
    R(1,3)+R(3,1)];

Z=  {[1 2 3]; [1 4 6]; [2 4 5]; [3 5 6]};

quat(1:4)=qmax;
quat(inds)=DD(Z{ii})/4/qmax;
end