%% Juan Heredia
%% Skoltech 2020
%% File used to calculate the Kinematics and Jacobian
%% KUKA IIWA LBR 7kg R800

clc
clear all
close all

syms q1 q2 q3 q4 q5 q6 q7 

l1 = 0.34;
l2 = 0.4
l3 = 0.4
l4 = 0.126



%% Art 1
teta=q1;
d=l1;
alfa=-90;
a=0;
A01 =[[ cos(teta) -cosd(alfa)*sin(teta)  sind(alfa)*sin(teta) a*cos(teta)];...
     [ sin(teta)  cosd(alfa)*cos(teta) -sind(alfa)*cos(teta) a*sin(teta)];...
     [          0             sind(alfa)             cosd(alfa)            d];...
     [          0                      0                     0,           1]];

%% Art 2
teta=q2;
d=0;
alfa=90;
a=0;
A12=[[ cos(teta) -cosd(alfa)*sin(teta)  sind(alfa)*sin(teta) a*cos(teta)];...
     [ sin(teta)  cosd(alfa)*cos(teta) -sind(alfa)*cos(teta) a*sin(teta)];...
     [          0             sind(alfa)             cosd(alfa)            d];...
     [          0                      0                     0,           1]];

%% Art 3
teta=q3;
d=l2;
alfa=90;
a=0;
A23=[[ cos(teta) -cosd(alfa)*sin(teta)  sind(alfa)*sin(teta) a*cos(teta)];...
     [ sin(teta)  cosd(alfa)*cos(teta) -sind(alfa)*cos(teta) a*sin(teta)];...
     [          0             sind(alfa)             cosd(alfa)            d];...
     [          0                      0                     0,           1]];
 
 %% Art 4
teta=q4;
d=0;
alfa=-90;
a=0;
A34=[[ cos(teta) -cosd(alfa)*sin(teta)  sind(alfa)*sin(teta) a*cos(teta)];...
     [ sin(teta)  cosd(alfa)*cos(teta) -sind(alfa)*cos(teta) a*sin(teta)];...
     [          0             sind(alfa)             cosd(alfa)            d];...
     [          0                      0                     0,           1]];

%% Art 5
teta=q5;
d=l3;
alfa=-90;
a=0;
A45=[[ cos(teta) -cosd(alfa)*sin(teta)  sind(alfa)*sin(teta) a*cos(teta)];...
     [ sin(teta)  cosd(alfa)*cos(teta) -sind(alfa)*cos(teta) a*sin(teta)];...
     [          0             sind(alfa)             cosd(alfa)            d];...
     [          0                      0                     0,           1]];
 
 
%% Art  6
teta=q6;
d=0;
alfa=90;
a=0;
A56=[[ cos(teta) -cosd(alfa)*sin(teta)  sind(alfa)*sin(teta) a*cos(teta)];...
     [ sin(teta)  cosd(alfa)*cos(teta) -sind(alfa)*cos(teta) a*sin(teta)];...
     [          0             sind(alfa)             cosd(alfa)            d];...
     [          0                      0                     0,           1]];

%% Art 7
teta=q7;
d=l4;
alfa=0;
a=0;
A67=[[ cos(teta) -cosd(alfa)*sin(teta)  sind(alfa)*sin(teta) a*cos(teta)];...
     [ sin(teta)  cosd(alfa)*cos(teta) -sind(alfa)*cos(teta) a*sin(teta)];...
     [          0             sind(alfa)             cosd(alfa)            d];...
     [          0                      0                     0,           1]];
 
%% Homogeneous Matrix
T=A01*A12*A23*A34*A45*A56*A67;
h=T(1:1:3,4);


%%
q1 = 0;
q2 = 0;
q3 = 0;
q4 = 0;
q5 = 0;
q6 = pi/2;
q7 = 0;

eval(h)
%%


%% Jacobian
syms alfa teta q1 q2 q3 q4 q5 q6 q7 

xe=T(1,4);
ye=T(2,4);
ze=T(3,4);
he=[xe ye ze];
qe=[q1 q2 q3 q4 q5 q6 q7]; 
for i=1:3
    for j=1:7
        JA(i,j)=diff(he(i),qe(j));
    end
end

%% Comprove the model with the simulation 

lbr = importrobot('iiwa7.urdf'); % 14 kg payload version
lbr.DataFormat = 'row';
gripper = 'iiwa_link_ee_kuka';

q = [pi/6,pi/2,pi/2,pi/2,pi/2,pi/6,pi/2];
gripperPosition = tform2trvec(getTransform(lbr,q,gripper))

q1 = q(1);
q2 = q(2);
q3 = q(3);
q4 = q(4);
q5 = q(5);
q6 = q(6);
q7 = q(7);

eval(h)

%% Save the Jacobian, matrix and end efector position

save('Kinematics.mat','JA','T','h')


%% Kinematics Other robot parts

