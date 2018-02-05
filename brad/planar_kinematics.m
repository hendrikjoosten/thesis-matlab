clc;clear

syms l1 l2 l3  th1 th2 th3  dth1 dth2 dth3 ph1 ph2 ph3  dph1 dph2 dph3 rh1 rh2 rh3  drh1 drh2 drh3

%the gen coordinates (used in finding velocity of point)
q=[th1 ph1 rh1 th2 ph2 rh2 th3 ph3 rh3];
dq=[dth1 dph1 drh1 dth2 dph2 drh2 dth3 dph3 drh3];

%rotation matrices for each point
Rpitch1=[[cos(th1) 0 sin(th1)];[0 1 0];[-sin(th1) 0 cos(th1)]];
Ryaw1=[[cos(ph1) -sin(ph1) 0];[sin(ph1) cos(ph1) 0];[0 0 1]];
Rroll1 = [[1 0 0];[0 cos(rh1) -sin(rh1)]; [0 sin(rh1) cos(rh1)]];

Rpitch2=[[cos(th2) 0 sin(th2)];[0 1 0];[-sin(th2) 0 cos(th2)]];
Ryaw2=[[cos(ph2) -sin(ph2) 0];[sin(ph2) cos(ph2) 0];[0 0 1]];
Rroll2 = [[1 0 0];[0 cos(rh2) -sin(rh2)]; [0 sin(rh2) cos(rh2)]];

Rpitch3=[[cos(th3) 0 sin(th3)];[0 1 0];[-sin(th3) 0 cos(th3)]];
Ryaw3=[[cos(ph3) -sin(ph3) 0];[sin(ph3) cos(ph3) 0];[0 0 1]];
Rroll3 = [[1 0 0];[0 cos(rh3) -sin(rh3)]; [0 sin(rh3) cos(rh3)]];


R_0_1=Ryaw1*Rroll1*Rpitch1;%Ryaw*Rroll*Rpitch;%231 (pitch, roll, yaw)
R_0_2=Ryaw2*Rroll2*Rpitch2;
R_0_3=Ryaw3*Rroll3*Rpitch3;
%R_0_1 (rotates body (1) to inertial (0))


P1=R_0_1*[l1;0;0]
P2=P1 + R_0_2*[l2;0;0]
P3=P2 + R_0_3*[l3;0;0]

 dP1=jacobian(P1,q)%*dq % differentiating
% dP2=jacobian(P2,q)*dq
% dP3=jacobian(P3,q)*dq

%l1=1
%th1=0.1

%subs(P1)