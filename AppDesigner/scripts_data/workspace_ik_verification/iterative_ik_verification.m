% attempt to find the catching line for the robot
% loads the robot and plots its path along a set trajectory


%% load model 
close all 
robot=importrobot('ARbot_Rev2_1.urdf');
% load robot
newPose = robot.randomConfiguration;
% Add another massless coordinate frame for the end effector
eeOffset = 0.150;
eeBody = robotics.RigidBody('end_effector');
eeBody.Mass = 0;
eeBody.Inertia = [0 0 0 0 0 0];
setFixedTransform(eeBody.Joint,trvec2tform([eeOffset 0 0]));
figure(1);
addBody(robot,eeBody,'L4');
axes = show(robot,robot.homeConfiguration);
axes.CameraPositionMode = 'auto';
%% 

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
% get joint angles for each point
P_end = P_A;
P_end(1) = -P_end(1);
P_mid_75 = midpoint(P_A,P_end);

%% 8
P_B = [ 0; 0; 0; 1;];
T_A_B(1:3,1:3) = base_to_catch_out;
T_A_B(1:3,4) = P_A_BORG_8;
P_A = T_A_B * P_B; % vector from the base of the robot to the target
% get joint angles for each point
P_end = P_A;
P_end(1) = -P_end(1);
P_mid_8 = midpoint(P_A,P_end);

%% 9
P_B = [ 0; 0; 0; 1;];
T_A_B(1:3,1:3) = base_to_catch_out;
T_A_B(1:3,4) = P_A_BORG_9;
P_A = T_A_B * P_B; % vector from the base of the robot to the target

% get joint angles for each point
P_end = P_A;
P_end(1) = -P_end(1);
P_mid_9 = midpoint(P_A,P_end);
%%
angleInRadians = deg2rad(-70);
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

%% Perform Inverse Kinematics No Constraints
% Use desired weights for solution (First three are orientation, last three are translation)
% Since it is a 4-DOF robot with only one revolute joint in Z we do not
% put a weight on Z rotation; otherwise it limits the solution space

ik = robotics.InverseKinematics('RigidBodyTree',robot);
weights = [0.1 0.1 0 1 1 1];
initialguess = robot.homeConfiguration;

% Call inverse kinematics solver for every end-effector position using the
% previous configuration as initial guess
for idx = 1:size(wayPoints,1)
    tform = trvec2tform(wayPoints(idx,:));
    configSoln(idx,:) = ik('end_effector',tform,weights,initialguess);
    initialguess = configSoln(idx,:);
end

%% Visualize robot configurations
title('Robot waypoint tracking visualization')
for idx = 1:size(wayPoints,1)
    % figure(idx);
    show(robot,configSoln(idx,:), 'PreservePlot', true,'Frames','off');
    pause(0.1)
    hold on
end


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