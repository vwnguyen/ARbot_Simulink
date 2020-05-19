% attempt to find the catching line for the robot
% loads the robot and plots its path along a set trajectory
% clear all
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
newPose = robot.randomConfiguration;

base_to_catch_out = [  0.0000    0.9397   -0.3420;
   -1.0000    0.0000         0;
         0    0.3420    0.9397];
ROTZ = rotz_d( 90 ); 
rotation_matrix = base_to_catch_out;
eeOrientation = deg2rad(-70);
T_A_B = [ 0 0 0 0;
  0 0 0 0;
  0 0 0 0;
  0 0 0 1;
  ];

%% generate arc from the corner and midpoints


corn = P_A_BORG_65;
L_corn = P_A_BORG_65;
R_corn = [ -P_A_BORG_65(1) P_A_BORG_65(2)  P_A_BORG_65(3)];

wayPoints = [L_corn ;
       R_corn ;
      P_mid_8;
      P_mid_9;
     % higher_mid;
      ] ;
% [theta theta_dot theta_ddot] = traj6_v2(ikSolStart,ikSolMid,ikSolEnd,2,1);
exampleHelperPlotWaypoints(wayPoints)
hold on

ikSolStart = inverseKineRBT(L_corn(1),L_corn(2),L_corn(3),eeOrientation);
ikSolEnd = inverseKineRBT(R_corn(1),R_corn(2),R_corn(3),eeOrientation);
ikSolMid = inverseKineRBT(P_mid_8(1),P_mid_8(2),P_mid_8(3),eeOrientation);
ikSolHigherMid = inverseKineRBT(P_mid_9(1),P_mid_9(2),P_mid_9(3),eeOrientation);


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


%% fit and plot

[center,rad,v1,v2] = circlefit3d(wayPoints(1,:),wayPoints(2,:),wayPoints(3,:));
idx = 1;
for i=1:1:361
    a = i/180*pi;
    x = center(:,1)+sin(a)*rad.*v1(:,1)+cos(a)*rad.*v2(:,1);
    y = center(:,2)+sin(a)*rad.*v1(:,2)+cos(a)*rad.*v2(:,2);
    z = center(:,3)+sin(a)*rad.*v1(:,3)+cos(a)*rad.*v2(:,3);
    workspace_points(idx,:) = [ x y z 1] ;
    plot3(x,y,z,'r.');
    idx = idx + 1;
end

%% filter workspace to relevant parts
indices_of_workspace = find(workspace_points(:,3)>corn(3));
filtered_workspace = workspace_points(indices_of_workspace,:);
filtered_workspace = sortrows(filtered_workspace);
for i=1:length(indices_of_workspace)
    a = i/180*pi;
    x = filtered_workspace(i,1);
    y = filtered_workspace(i,2);
    z = filtered_workspace(i,3);
    plot3(x,y,z,'g.');
    pause(0.01);
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
    pause(0.01);
    idx = idx + 1;
end

%% plot filtered (in front of robot)
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
    pause(0.01);
    idx = idx + 1;
end

figure(1)
ikLookup = [];
for i = 1:size(P_A,1)
    [theta] = inverseKineRBT(filtered_workspace(i,1),filtered_workspace(i,2),filtered_workspace(i,3),eeOrientation);
    ikLookup(i,:) = theta;
    newPose(1).JointPosition = theta(1);
    newPose(2).JointPosition = theta(2);
    newPose(3).JointPosition = theta(3);
    newPose(4).JointPosition = theta(4);
    tform = getTransform(robot,newPose,'L4');
    show(robot,newPose, 'PreservePlot', true,'Frames','off');
    hold on
    pause(0.01);
    % robot.plot(theta(idx));
end
hold off
% %% plot corrected, to side of robot
% figure(2)
% ikLookup = [];
% for i = 1:size(P_A,1)
%     [theta] = inverseKineRBT(corrected_workspace(i,1),corrected_workspace(i,2),corrected_workspace(i,3),eeOrientation);
%     ikLookup(i,:) = theta;
%     newPose(1).JointPosition = theta(1);
%     newPose(2).JointPosition = theta(2);
%     newPose(3).JointPosition = theta(3);
%     newPose(4).JointPosition = theta(4);
%     tform = getTransform(robot,newPose,'L4');
%     show(robot,newPose, 'PreservePlot', true,'Frames','off');
%     hold on
%     pause(0.01);
%     % robot.plot(theta(idx));
% end
% hold off
% %% plot to other side of robot 180 deg Z rot
% figure(3)
% 
% P_A_BORG = [[0];[0];[0];]; % middle catch line, use 
% P_B = [];
% P_A = [];
% for i=1:length(filtered_workspace)
%     P_B(i,:) = [ filtered_workspace(i,1) filtered_workspace(i,2) filtered_workspace(i,3) 1];
%     T_A_B(1:3,1:3) = rotz_d(180);
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
% 
% figure(3)
% ikLookup = [];
% for i = 1:size(P_A,1)
%     [theta] = inverseKineRBT(P_A(i,1),P_A(i,2),P_A(i,3),eeOrientation);
%     ikLookup(i,:) = theta;
%     newPose(1).JointPosition = theta(1);
%     newPose(2).JointPosition = theta(2);
%     newPose(3).JointPosition = theta(3);
%     newPose(4).JointPosition = theta(4);
%     tform = getTransform(robot,newPose,'L4');
%     show(robot,newPose, 'PreservePlot', true,'Frames','off');
%     hold on
%     pause(0.01);
%     % robot.plot(theta(idx));
% end
% hold off
% %% save workspace in terms of 3D points and save the ik solutions
% 
% save('ikLookup.mat','ikLookup');
% save('ARbot_workspace.mat','corrected_workspace');

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
