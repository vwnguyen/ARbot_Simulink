% Create a Four link manipulator in MATLAB with Robotics System Toolbox
% Compute Forward Kinematics, Jacobian, and Inverse Kinematics


%% Define robot constant parameters
clear;
FourLinkRobot.l1=0.2;
FourLinkRobot.l2=0.5;
FourLinkRobot.l3=0.4;
FourLinkRobot.l4=0.3;

%% Create robot main structure
robot = robotics.RigidBodyTree();

%% Define DH parameters for defining the transformations between each link
%           a       alpha   d     theta
dhparams = [0   	pi/2	0.2   	0;
            0.5	      0     0       0;
            0.4	      0	    0	    0;
            0.3   	  0     0       0];

%% Create link 1 and attach joint
link1=robotics.RigidBody('L1');
jnt1=robotics.Joint('jnt1','revolute');
jnt1.PositionLimits=deg2rad([-180,180]);
%setFixedTransform(jnt1,[eye(3),[0;0;0];[0 0 0 1]]);
setFixedTransform(jnt1,dhparams(1,:),'dh');
jnt1.JointAxis=[0 0 1];
link1.Joint=jnt1;
%
addBody(robot,link1,'base');

%% Create link 2 and attach joint
link2=robotics.RigidBody('L2');
jnt2=robotics.Joint('jnt2','revolute');
jnt2.PositionLimits=deg2rad([-180,180]);
%setFixedTransform(jnt2,[eye(3),[FourLinkRobot.l1;0;0];[0 0 0 1]]);
setFixedTransform(jnt2,dhparams(2,:),'dh');
jnt2.JointAxis=[0 0 1];
link2.Joint=jnt2;
%
addBody(robot,link2,'L1');

%% Create link 3 and attach joint
link3=robotics.RigidBody('L3');
jnt3=robotics.Joint('jnt3','revolute');
jnt3.PositionLimits=deg2rad([-180,180]);
%setFixedTransform(jnt3,[eye(3),[FourLinkRobot.l2;0;0];[0 0 0 1]]);
setFixedTransform(jnt3,dhparams(3,:),'dh');
jnt3.JointAxis=[0 0 1];
link3.Joint=jnt3;
%
addBody(robot,link3,'L2');

%% Create link 4 and attach joint
link4=robotics.RigidBody('L4');
jnt4=robotics.Joint('jnt4','revolute');
jnt4.PositionLimits=deg2rad([-180,180]);
%setFixedTransform(jnt4,[eye(3),[FourLinkRobot.l3;0;0];[0 0 0 1]]);
setFixedTransform(jnt4,dhparams(4,:),'dh');
jnt4.JointAxis=[0 0 1];
link4.Joint=jnt4;
%
addBody(robot,link4,'L3');

%% Create joint and link for end effector
% link5=robotics.RigidBody('L5');
% jnt5=robotics.Joint('jnt5','fixed');
% setFixedTransform(jnt5,[eye(3),[FourLinkRobot.l4;0;0];[0 0 0 1]]);
% link5.Joint=jnt5;
% %
% addBody(robot,link5,'L4');

%% Show details of kinematic chain
showdetails(robot);

%% Show figure of robot in it home configuration
Qhome=robot.homeConfiguration;
qHome=[Qhome.JointPosition];
figure(1);
mySol=[pi/4 0 0 0];
Qhome(1,1).JointPosition=mySol(1);
Qhome(1,2).JointPosition=mySol(2);
Qhome(1,3).JointPosition=mySol(3);
Qhome(1,4).JointPosition=mySol(4);
show(robot,Qhome);

%% Calculate Forward Kinematics and Jacobian
% Q=robot.randomConfiguration();
% q=[Q.JointPosition];
% figure(1);
% show(robot, Q);
% T=getTransform(robot,Q,'L4');
% J=geometricJacobian(robot,Q,'L4');
% 
% %% Configure Inverse Kinematics
% ik=robotics.InverseKinematics('RigidBodyTree', robot);
% weights=[1 1 1 1 1 1];
% ikInitialGuess=robot.homeConfiguration;
% q=[pi/2 pi/2 pi/2 pi/2];
% ikInitialGuess=arrayfun(@(x,y) setfield(x,'jointPosition',y),ikInitialGuess,q);
% 
% %% Solve Inverse Kinematics
% % set end effector pose
% Tee=eul2tform([pi/4 0 0]); % HTM rotation matrix (ZYX sequence) ex: eul2tform(zVar,yVar,xVar)
% Tee(1:3,4)=[0.5*(FourLinkRobot.l1+FourLinkRobot.l2+FourLinkRobot.l3+FourLinkRobot.l4) .3 FourLinkRobot.l1]; % pos to solve for
% [QSol,SolInfo]=step(ik,'L4',Tee,weights,ikInitialGuess);
% mySol=inverseKineRBT(Tee(1,4),Tee(2,4),Tee(3,4),-pi/4);
% Q(1,1).JointPosition=mySol(1);
% Q(1,2).JointPosition=mySol(2);
% Q(1,3).JointPosition=mySol(3);
% Q(1,4).JointPosition=mySol(4);
% 
% figure(1);
% %show(robot,QSol); hold on;
% show(robot,Q);
% %hold off;
% qSol=[QSol.JointPosition]';
% % Verify solution by comparing Tee(1:3,4) with Tik(1:3,4)
% Tik=getTransform(robot,QSol,'L4')
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
%{
Notes
1. function forwardKineRBT reproduces the results from getTransform(robot,QSol,'L4')
  i.e. forwardKineRBT(qSol(1),qSol(2),qSol(3),qSol(4)) == getTransform(robot,QSol,'L4')
2. inverseKineRBT()achieves a different joint configuration but does
achieve the same result as the matlab inverse kinematic algorithm
%}


