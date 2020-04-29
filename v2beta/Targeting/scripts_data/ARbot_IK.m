clear all;
%% add urdf file to path
%addpath('C:\Users\logan\Documents\MATLAB\Robotics\Arbot_v1\description\urdf');
rbot=importrobot('arbot_v1.urdf');
%% add end effector frame
eeOffset = 0.3;
eeBody = robotics.RigidBody('ee');
eeBody.Mass = 0;
eeBody.Inertia = [0 0 0 0 0 0];
setFixedTransform(eeBody.Joint,trvec2tform([eeOffset 0 0]));
addBody(rbot,eeBody,'L4');
figure(1);
axes=show(rbot);
%% test forward kinematics
Q=randomConfiguration(rbot);
Q(1,1).JointPosition=0;
Q(1,2).JointPosition=pi/4;
Q(1,3).JointPosition=-pi/4;
Q(1,4).JointPosition=-pi/4;
T=getTransform(rbot,Q,'ee');
% if thereT=myT then forward kinematics works
thereT=T(1:3,4)';
q=[Q.JointPosition];
myT=forwardKineRBT(q(1),q(2),q(3),q(4))';
show(rbot,Q); hold on;

% test if inverse kinematics agree
target=myT
q=[Q.JointPosition];
%% Configure Inverse Kinematics
ik=robotics.InverseKinematics('RigidBodyTree', rbot);
weights=[1 1 1 1 1 1];
ikInitialGuess=rbot.homeConfiguration;
qGuess=[pi/2 pi/2 pi/2 pi/2];
ikInitialGuess=arrayfun(@(x,y) setfield(x,'jointPosition',y),ikInitialGuess,qGuess);
% 
%% Solve Inverse Kinematics
% set end effector pose
phi=pi/4;
Tee=eul2tform([phi 0 0]); % HTM rotation matrix (ZYX sequence) ex: eul2tform(zVar,yVar,xVar)
Tee(1:3,4)=target; % pos to solve for
[thereSol,SolInfo]=step(ik,'L4',Tee,weights,ikInitialGuess);
mySol=inverseKineRBT(target(1),target(2),target(3),phi);
Q(1,1).JointPosition=mySol(1);
Q(1,2).JointPosition=mySol(2);
Q(1,3).JointPosition=mySol(3);
Q(1,4).JointPosition=mySol(4);

compare=[[thereSol(1,:).JointPosition]' mySol'];
%plot resulting pose
show(rbot,Q);hold off;
figure(2)
show(rbot,Q); 
% 
% figure(1);
% %show(robot,QSol); hold on;
% show(robot,Q);
% %hold off;
% qSol=[QSol.JointPosition]';
% Verify solution by comparing Tee(1:3,4) with Tik(1:3,4)
Tik=getTransform(rbot,thereSol,'ee');
q=[Q.JointPosition];
forwardKineRBT(q(1),q(2),q(3),q(4))'
% 
% %% Verify with my functions
% fk_diff=floor(abs(Tik(1:3,4)-forwardKineRBT(qSol(1),qSol(2),qSol(3),qSol(4))))
% ik_diff=rad2deg(qSol-inverseKineRBT(Tee(1,4),Tee(2,4),Tee(3,4),pi/4)')
% mySol=inverseKineRBT(Tee(1,4),Tee(2,4),Tee(3,4),pi/4);
% Q(1,1).JointPosition=mySol(1);
% Q(1,2).JointPosition=mySol(2);
% Q(1,3).JointPosition=mySol(3);
% Q(1,4).JointPosition=mySol(4);
% pos=getTransform(robot,Q,'L4')




