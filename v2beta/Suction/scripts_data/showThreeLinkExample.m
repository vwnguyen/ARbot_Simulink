% Create a Three link manipulator in MATLAB with Robotics System Toolbox
% Compute Forward Kinematics, Jacobian, and Inverse Kinematics
% Credit: Carlos Santacruz-Rosero, Ph.D.

%% Define robot constant parameters
clear;
ThreeLinkRobot.l1=0.3;
ThreeLinkRobot.l2=0.2;
ThreeLinkRobot.l3=0.1;


%% Create robot main structure
robot = robotics.RigidBodyTree();

%% Create link 1 and attach joint
link1=robotics.RigidBody('L1');
jnt1=robotics.Joint('jnt1','revolute');
jnt1.PositionLimits=deg2rad([-180,180]);
setFixedTransform(jnt1,[eye(3),[0;0;0];[0 0 0 1]]);
jnt1.JointAxis=[0 1 0];
link1.Joint=jnt1;
%
addBody(robot,link1,'base');

%% Create link 2 and attach joint
link2=robotics.RigidBody('L2');
jnt2=robotics.Joint('jnt2','revolute');
jnt2.PositionLimits=deg2rad([-180,180]);
setFixedTransform(jnt2,[eye(3),[ThreeLinkRobot.l1;0;0];[0 0 0 1]]);
jnt2.JointAxis=[0 1 0];
link2.Joint=jnt2;
%
addBody(robot,link2,'L1');

%% Create link 3 and attach joint
link3=robotics.RigidBody('L3');
jnt3=robotics.Joint('jnt3','revolute');
jnt3.PositionLimits=deg2rad([-180,180]);
setFixedTransform(jnt3,[eye(3),[ThreeLinkRobot.l2;0;0];[0 0 0 1]]);
jnt3.JointAxis=[0 1 0];
link3.Joint=jnt3;
%
addBody(robot,link3,'L2');

%% Create joint and link for end effector
link4=robotics.RigidBody('L4');
jnt4=robotics.Joint('jnt4','fixed');
setFixedTransform(jnt4,[eye(3),[ThreeLinkRobot.l3;0;0];[0 0 0 1]]);
link4.Joint=jnt4;
%
addBody(robot,link4,'L3');

%% Show details of kinematic chain
showdetails(robot);

%% Show figure of robot in it home configuration
Qhome=robot.homeConfiguration;
qHome=[Qhome.JointPosition];
figure(1);
show(robot,Qhome);

%% Calculate Forward Kinematics and Jacobian
Q=robot.randomConfiguration();
q=[Q.JointPosition];
figure(1);
show(robot, Q);
T=getTransform(robot,Q,'L4');
J=geometricJacobian(robot,Q,'L4');

%% Configure Inverse Kinematics
ik=robotics.InverseKinematics('RigidBodyTree', robot);
weights=[1 1 1 1 1 1];
ikInitialGuess=robot.homeConfiguration;
q=[pi/2 pi/2 pi/2];
ikInitialGuess=arrayfun(@(x,y) setfield(x,'jointPosition',y),ikInitialGuess,q);

%% Solve Inverse Kinematics
% set end effector pose
Tee=eul2tform([0 pi/4 0]); % HTM rotation matrix (ZYX sequence)
Tee(1:3,4)=[0 0 0.5*(ThreeLinkRobot.l1+ThreeLinkRobot.l2+ThreeLinkRobot.l3)]; % pos to solve for
[QSol,SolInfo]=step(ik,'L4',Tee,weights,ikInitialGuess);

figure(1);
show(robot,QSol);
qSol=[QSol.JointPosition]';
% Verify solution
Tik=getTransform(robot,QSol,'L4')



