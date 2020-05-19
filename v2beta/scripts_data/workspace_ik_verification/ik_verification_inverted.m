% attempt to find the catching line for the robot
% loads the robot and plots its path along a set trajectory

% run startup_Arbot_Dim.m first 
% then run generate_workspace.m

close all 
% environment = 'C:\Users\Viet\Desktop\git_repos\ARbot_Simulink\v2offset\Targeting\Environment'
% addpath(genpath(environment));
% % add the end effector
robot=importrobot('ARbot_Rev2_1.urdf');
% load robot
newPose = robot.randomConfiguration;
figure(1)
axes = show(robot,robot.homeConfiguration);
axes.CameraPositionMode = 'auto';

base_to_catch_out = [  0.0000    0.9397   -0.3420;
   -1.0000    0.0000         0;
         0    0.3420    0.9397];
ROTZ = rotz( 90 ); 
rotation_matrix = base_to_catch_out;
     
T_A_B = [ 0 0 0 0;
  0 0 0 0;
  0 0 0 0;
  0 0 0 1;
  ];

%% old catching line ( NOT FLIPPED ) 
P_A_BORG_65 = [ 0.2205 0.5588 -0.3135];
P_A_BORG_75 = [0.5024 0.5588 -0.2109];
P_A_BORG_8 = [0.6434 0.5588 -0.1596];
P_A_BORG_9 = [0.9253 0.5588 -0.0570];

% FLIP X AND Y TO ACCOUNT FOR MATLAB REFERENCE FRAME
P_A_BORG_65 = [ -0.5588 -0.2205 -0.3135];
P_A_BORG_75= [-0.5588 -0.5024 -0.2109]; % 2nd furthest 9.75
P_A_BORG_8 = [ -0.5588; -0.6434; -0.1596 ]; % furthest from the robot 9.8
P_A_BORG_9 = [ -0.5588; -0.9253; -0.0570 ]; % furthest from the robot 9.9

% corners for the end point

%% 75
P_B = [ 0; 0; 0; 1;];
T_A_B(1:3,1:3) = base_to_catch_out;
T_A_B(1:3,4) = P_A_BORG_75;
P_A = T_A_B * P_B; % vector from the base of the robot to the target
angleInRadians = deg2rad(-70);
% get joint angles for each point
P_end = P_A;
P_end(1) = -P_end(1);
P_mid_75 = midpoint(P_A,P_end);

%% 8
P_B = [ 0; 0; 0; 1;];
T_A_B(1:3,1:3) = base_to_catch_out;
T_A_B(1:3,4) = P_A_BORG_8;
P_A = T_A_B * P_B; % vector from the base of the robot to the target
angleInRadians = deg2rad(-70);
% get joint angles for each point
P_end = P_A;
P_end(1) = -P_end(1);
P_mid_8 = midpoint(P_A,P_end);

%% 9
P_B = [ 0; 0; 0; 1;];
T_A_B(1:3,1:3) = base_to_catch_out;
T_A_B(1:3,4) = P_A_BORG_9;
P_A = T_A_B * P_B; % vector from the base of the robot to the target
angleInRadians = deg2rad(-70);
% get joint angles for each point
P_end = P_A;
P_end(1) = -P_end(1);
P_mid_9 = midpoint(P_A,P_end);
%%

wayPoints = [ P_A_BORG_65(1) P_A_BORG_65(2) P_A_BORG_65(3);
   -P_A_BORG_65(1) P_A_BORG_65(2) P_A_BORG_65(3);
   
      P_mid_75;
      P_mid_8;
      P_mid_9;] ;
exampleHelperPlotWaypoints(wayPoints)
hold on

% create line to add points to
h1 = animatedline('LineWidth',5,'Color','r');
pos = [];
end_effector_pos = [];
%% generate arc from the corner and midpoints
ikSolCorn = inverseKineRBT(P_A_BORG_65(1),P_A_BORG_65(2),P_A_BORG_65(3),angleInRadians);
ikSolCorn2 = inverseKineRBT(-P_A_BORG_65(1),P_A_BORG_65(2),P_A_BORG_65(3),angleInRadians);
ikSolStart = inverseKineRBT(P_mid_75(1),P_mid_75(2),P_mid_75(3),angleInRadians);
ikSolEnd = inverseKineRBT(P_mid_8(1),P_mid_8(2),P_mid_8(3),angleInRadians);
ikSolMid = inverseKineRBT(P_mid_9(1),P_mid_9(2),P_mid_9(3),angleInRadians);


%% show robot attaining defined waypoints 
newPose(1).JointPosition = ikSolStart(1);
newPose(2).JointPosition = ikSolStart(2);
newPose(3).JointPosition = ikSolStart(3);
newPose(4).JointPosition = ikSolStart(4);
show(robot,newPose, 'PreservePlot', true,'Frames','off');
newPose(1).JointPosition = ikSolMid(1);
newPose(2).JointPosition = ikSolMid(2);
newPose(3).JointPosition = ikSolMid(3);
newPose(4).JointPosition = ikSolMid(4);
show(robot,newPose, 'PreservePlot', true,'Frames','off');
newPose(1).JointPosition = ikSolEnd(1);
newPose(2).JointPosition = ikSolEnd(2);
newPose(3).JointPosition = ikSolEnd(3);
newPose(4).JointPosition = ikSolEnd(4);
show(robot,newPose, 'PreservePlot', true,'Frames','off');
newPose(1).JointPosition = ikSolCorn(1);
newPose(2).JointPosition = ikSolCorn(2);
newPose(3).JointPosition = ikSolCorn(3);
newPose(4).JointPosition = ikSolCorn(4);
show(robot,newPose, 'PreservePlot', true,'Frames','off');
newPose(1).JointPosition = ikSolCorn2(1);
newPose(2).JointPosition = ikSolCorn2(2);
newPose(3).JointPosition = ikSolCorn2(3);
newPose(4).JointPosition = ikSolCorn2(4);
show(robot,newPose, 'PreservePlot', true,'Frames','off');




%% Functions

function rot_z = rotz(ang)

    rot_z = [ cosd(ang) -sind(ang) 0;
    sin(ang) cos(ang) 0 ;
    0 0 1;];
end

function P_mid = midpoint(P1,P2)
    P_mid(1) = (P1(1) + P2(1)) / 2;
    P_mid(2) = (P1(2) + P2(2)) / 2;
    P_mid(3) = (P1(3) + P2(3)) / 2;
end