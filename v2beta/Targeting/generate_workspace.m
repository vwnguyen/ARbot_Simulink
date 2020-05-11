% attempt to find the catching line for the robot
% loads the robot and plots its path along a set trajectory
% clear all
clear 
close all 
environment = 'C:\Users\Viet\Desktop\git_repos\ARbot_Simulink\v2beta\Environment\PlantModel';
addpath(genpath(environment));
% add the end effector
robot=importrobot('arbot_betav1.urdf');
% load robot
newPose = robot.randomConfiguration;
figure(1)
rotate3d(figure(1))
axes = show(robot,robot.homeConfiguration);
axes.CameraPositionMode = 'auto';

base_to_catch_out = [  0.0000    0.9397   -0.3420;
   -1.0000    0.0000         0;
         0    0.3420    0.9397];
ROTZ = rotz_d( 90 ); 
rotation_matrix = base_to_catch_out;
     
T_A_B = [ 0 0 0 0;
  0 0 0 0;
  0 0 0 0;
  0 0 0 1;
  ];

%% generate arc from the corner and midpoints

mdpt = load('P_mid','P_mid');
L_corner = load('P_end','P_end');

mid = mdpt.P_mid;
corn = L_corner.P_end;
L_corn = [ corn(1) corn(2)  corn(3)];
R_corn = [ -corn(1) corn(2)  corn(3)];
%% 
wayPoints = [ corn(1) corn(2)  corn(3) ;
   -corn(1)  corn(2)  corn(3) ;
      mid  ] ;
% [theta theta_dot theta_ddot] = traj6_v2(ikSolStart,ikSolMid,ikSolEnd,2,1);
exampleHelperPlotWaypoints(wayPoints)
hold on

%% fit and plot

[center,rad,v1,v2] = circlefit3d(wayPoints(1,:),wayPoints(2,:),wayPoints(3,:));
idx = 1;
for i=0.5:0.5:361
    a = i/180*pi;
    x = center(:,1)+sin(a)*rad.*v1(:,1)+cos(a)*rad.*v2(:,1);
    y = center(:,2)+sin(a)*rad.*v1(:,2)+cos(a)*rad.*v2(:,2);
    z = center(:,3)+sin(a)*rad.*v1(:,3)+cos(a)*rad.*v2(:,3);
    workspace_points(idx,:) = [ x y z] ;
    plot3(x,y,z,'r.');
    idx = idx + 1;
end

%% filter workspace to relevant parts
indices_of_workspace = find(workspace_points(:,3)>corn(3));
filtered_workspace = workspace_points(indices_of_workspace,:);
for i=1:length(indices_of_workspace)
    a = i/180*pi;
    x = filtered_workspace(i,1);
    y = filtered_workspace(i,2);
    z = filtered_workspace(i,3);
    plot3(x,y,z,'g.');
    idx = idx + 1;
end
hold on
%% convert points from world to robot POV

corrected_workspace(:,1) = filtered_workspace(:,2);
corrected_workspace(:,2) = filtered_workspace(:,1);
corrected_workspace(:,3) = filtered_workspace(:,3);
% points are now corrected, to be in the robot POV

for i=1:length(indices_of_workspace)
    a = i/180*pi;
    x = corrected_workspace(i,1);
    y = corrected_workspace(i,2);
    z = corrected_workspace(i,3);
    plot3(x,y,z,'c.');
    idx = idx + 1;
end

%% FLIP X AND Y TO ACCOUNT FOR MATLAB REFERENCE FRAME
P_A_BORG = [[0];[0];[0];]; % middle catch line, use 
P_B = [];
P_A = [];
for i=1:length(filtered_workspace)
    P_B(i,:) = [ filtered_workspace(i,1) filtered_workspace(i,2) filtered_workspace(i,3) 1];
    T_A_B(1:3,1:3) = rotz_d(90);
    T_A_B(1:3,4) = P_A_BORG;
    P_A(i,:) = T_A_B * P_B(i,:).';
end

for i=1:length(P_A)
    x = P_A(i,1);
    y = P_A(i,2);
    z = P_A(i,3);
    plot3(x,y,z,'b.');
    idx = idx + 1;
end
%%
newPose = robot.randomConfiguration;
eeOrientation = deg2rad(-70);
for i = 1:size(P_A,1)
    [theta] = inverseKineRBT(filtered_workspace(i,1),filtered_workspace(i,2),filtered_workspace(i,3),eeOrientation)
    newPose(1).JointPosition = theta(1);
    newPose(2).JointPosition = theta(2);
    newPose(3).JointPosition = theta(3);
    newPose(4).JointPosition = theta(4);
    tform = getTransform(robot,newPose,'L4');
    show(robot,newPose, 'PreservePlot', true,'Frames','off');
    hold on
    % robot.plot(theta(idx));
    pause(0.01)
end

%% describe points relative to robot and solve for them

% P_A_BORG = [[0];[0];[0];]; % middle catch line, use 
% P_B = [];
% P_A = [];
% for i=1:length(filtered_workspace)
%     P_B(i,:) = [ filtered_workspace(i,1) filtered_workspace(i,2) filtered_workspace(i,3) 1];
%     T_A_B(1:3,1:3) = rotz_d(-90);
%     T_A_B(1:3,4) = P_A_BORG;
%     P_A(i,:) = T_A_B * P_B(i,:).';
% end
% 
% for i=1:length(P_A)
%     x = P_A(i,1);
%     y = P_A(i,2);
%     z = P_A(i,3);
%     plot3(x,y,z,'b.');
%     idx = idx + 1;
% end

%% Functions

function rot_z = rotz_d(ang)

    rot_z = [ cosd(ang) -sind(ang) 0;
    sind(ang) cosd(ang) 0 ;
    0 0 1;];
end

function P_mid = midpoint(P1,P2)
    P_mid(1) = (P1(1) + P2(1)) / 2;
    P_mid(2) = (P1(2) + P2(2)) / 2;
    P_mid(3) = (P1(3) + P2(3)) / 2;
end
