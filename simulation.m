%% Juan Heredia
%% Skoltech 2020
%% File used to calculate the Kinematics and Jacobian
%% KUKA IIWA LBR 7kg R800

clc
clear all
close all

lbr = importrobot('iiwa7.urdf'); % 14 kg payload version
lbr.DataFormat = 'row';
gripper = 'iiwa_link_ee_kuka';

%%
%lbr.DataFormat = 'row';
q = [pi/6,pi/2,pi/2,pi/2,pi/2,pi/6,pi/2];
gripperPosition = tform2trvec(getTransform(lbr,q,gripper))

%%

gik = robotics.GeneralizedInverseKinematics;
gik.RigidBodyTree = lbr;
gik.ConstraintInputs = {'position','aiming'};
posTgt = robotics.PositionTarget(gripper);
posTgt.TargetPosition = [0.0 0.25 1.0];
aimCon = robotics.AimingConstraint(gripper);
aimCon.TargetPoint = [0.0 0.0 0.0];
q0 = homeConfiguration(lbr); % Initial guess for solver
[q,solutionInfo] = gik(q0,posTgt,aimCon)
gripperPosition = tform2trvec(getTransform(lbr,q,gripper))
show(lbr,q)

