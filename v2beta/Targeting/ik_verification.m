% attempt to find the catching line for the robot
% loads the robot and plots its path along a set trajectory
environment = 'C:\Users\Viet\Desktop\git_repos\ARbot_Simulink\v2beta\Environment\PlantModel'
addpath(genpath(environment));
% add the end effector
robot=importrobot('arbot_betav1.urdf');
% load robot
newPose = robot.randomConfiguration;

jointAngles = ikSol;
newPose(1).JointPosition = jointAngles(1);
newPose(2).JointPosition = jointAngles(2);
newPose(3).JointPosition = jointAngles(3);
newPose(4).JointPosition = jointAngles(4);
figure(1)
axes = show(robot,newPose);
axes.CameraPositionMode = 'auto';

base_to_catch_out = [  0.0000    0.9397   -0.3420;
   -1.0000    0.0000         0;
         0    0.3420    0.9397];

rotation_matrix = base_to_catch_out;
     
T_A_B = [ 0 0 0 0;
  0 0 0 0;
  0 0 0 0;
  0 0 0 1;
  ];

P_A_BORG = [[0.4655];[0.5639];[ -0.2184]];

% where at the catching line it should end effector go 
P_B = [ 0; 0; 0; 1;];

T_A_B(1:3,1:3) = base_to_catch_out;
T_A_B(1:3,4) = P_A_BORG;
P_A = T_A_B * P_B; % vector from the base of the robot to the target


angleInRadians = deg2rad(-70);

% get joint angles for each point
ikSolStart = inverseKineRBT(P_A(1),P_A(2),P_A(3),angleInRadians)

P_end = P_A;
P_end(1) = -P_end(1);
P_mid = midpoint(P_A,P_end)

ikSolMid = inverseKineRBT(P_mid(1),P_mid(2),P_mid(3),angleInRadians)
ikSolEnd = inverseKineRBT(-P_A(1),P_A(2),P_A(3),angleInRadians)

wayPoints = [ P_A_BORG(1) P_A_BORG(2) P_A_BORG(3);
   -P_A_BORG(1) P_A_BORG(2) P_A_BORG(3);
      P_mid  ] ;
[theta theta_dot theta_ddot] = traj6_v2(ikSolStart,ikSolMid,ikSolEnd,2,1)
exampleHelperPlotWaypoints(wayPoints)
hold on

for idx = 1:size(theta,1)
    newPose(1).JointPosition = theta(idx,1);
    newPose(2).JointPosition = theta(idx,2);
    newPose(3).JointPosition = theta(idx,3);
    newPose(4).JointPosition = theta(idx,4);
    tform = getTransform(robot,newPose,'L4');
    show(robot,newPose, 'PreservePlot', true,'Frames','off');
    % robot.plot(theta(idx));
    pause(0.01)
end

function P_mid = midpoint(P1,P2)
    P_mid(1) = (P1(1) + P2(1)) / 2;
    P_mid(2) = (P1(2) + P2(2)) / 2;
    P_mid(3) = (P1(3) + P2(3)) / 2;
end
% plot3(trajectory(1,:),trajectory(2,:),trajectory(3,:),'r-','LineWidth',2);

% define trajectory


% plot i