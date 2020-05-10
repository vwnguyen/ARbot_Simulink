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
axes = show(robot,robot.homeConfiguration);
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

%% old catching line
% P_A_BORG = [[0.4655];[0.5639];[ -0.2184]];
P_A_BORG = [ 0.3615; 0.5588; -0.2622; ]; %closer to robot

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
P_mid = midpoint(P_A,P_end);

ikSolMid = inverseKineRBT(P_mid(1),P_mid(2),P_mid(3),angleInRadians);
ikSolEnd = inverseKineRBT(-P_A(1),P_A(2),P_A(3),angleInRadians);

wayPoints = [ P_A_BORG(1) P_A_BORG(2) P_A_BORG(3);
   -P_A_BORG(1) P_A_BORG(2) P_A_BORG(3);
      P_mid  ] ;
[theta theta_dot theta_ddot] = traj6_v2(ikSolStart,ikSolMid,ikSolEnd,2,1);
exampleHelperPlotWaypoints(wayPoints)
hold on

% create line to add points to
h1 = animatedline('LineWidth',5,'Color','r');
pos = [];
end_effector_pos = [];
%% Show the robot's movement
figure(1);
for idx = 1:size(theta,1)
    newPose(1).JointPosition = theta(idx,1);
    newPose(2).JointPosition = theta(idx,2);
    newPose(3).JointPosition = theta(idx,3);
    newPose(4).JointPosition = theta(idx,4);
    
    pos = forwardKineRBT(theta(idx,1), theta(idx,2),theta(idx,3),theta(idx,4));
    end_effector_pos(idx,:) = pos;
    addpoints(h1,pos(1),pos(2),pos(3));
    drawnow
    
    tform = getTransform(robot,newPose,'L4');
    show(robot,newPose, 'PreservePlot', true,'Frames','off');
    % robot.plot(theta(idx));
    pause(0.01)
end

%% Plot trajectory in 3D space from robot POV
% figure(2);
h2 = animatedline('LineWidth',5,'Color','g','Marker','+');
% axis([-0.5 0.5 0.5 0.575 -.25 -.2]) % new axis
% new_catching_line = fit([end_effector_pos(:,1),end_effector_pos(:,2)],end_effector_pos(:,3),'poly11')
for point = 1:length(end_effector_pos)
    addpoints(h2,end_effector_pos(point,1),end_effector_pos(point,2),end_effector_pos(point,3));
    drawnow
    xlabel('X')
    ylabel('Y')
    zlabel('Z') 
end

% show the robot and the mapped points

%% Map End Effector positions to Catching Line (P_B)
P_B_AORG = [ 0.5588;  -0.2500;  0.3700; ]; %closer to robot % reverse of atob
T_A_B_inv = inv(T_A_B);
rotation_matrix = [ 0.0000   -1.0000         0;
                    0.9397    0.0000    0.3420;
                   -0.3420         0    0.9397;]; 
T_B_A = [ 0 0 0 0;
  0 0 0 0;
  0 0 0 0;
  0 0 0 1;
  ];
% where at the base of the robot
P_A = [ 0; 0; 0; 1;];
T_B_A(1:3,1:3) = rotation_matrix; % reverse of A to B
T_B_A(1:3,4) = P_B_AORG;
P_B = T_B_A * P_A; % vector from the base of the robot to the target

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% base_to_catch_out = [  0.0000    0.9397   -0.3420;
%    -1.0000    0.0000         0;
%          0    0.3420    0.9397];
% 
% rotation_matrix = base_to_catch_out;
%      
% T_A_B = [ 0 0 0 0;
%   0 0 0 0;
%   0 0 0 0;
%   0 0 0 1;
%   ];
% 
% P_A_BORG = [ 0.3615; 0.5588; -0.2622; ]; %closer to robot
% 
% % where at the catching line it should end effector go 
% P_B = [ 0; 0; 0; 1;];
% 
% T_A_B(1:3,1:3) = base_to_catch_out;
% T_A_B(1:3,4) = P_A_BORG;
% P_A = T_A_B * P_B; % vector from the base of the robot to the target




p_b_mapped = [];
end_effector_pos(:,4) = ones(length(end_effector_pos),1); 
catching_line_matrix = [];
for i = 1:length(end_effector_pos)
    p_b_mapped = T_A_B_inv * end_effector_pos(i,:)';
    % p_b_mapped(3) = 0;
    catching_line_matrix(i,:) = p_b_mapped;
end

%% plot the mapped points for confirmation

figure(3);
axes = show(robot,robot.homeConfiguration);
exampleHelperPlotWaypoints(wayPoints);
axes.CameraPositionMode = 'auto';
hold on
h3 = animatedline('LineWidth',5,'Color','b','Marker','+');
for point = 1:length(catching_line_matrix)
    
    verified_arc(point,:) = T_A_B * catching_line_matrix(point,:)'
    
    addpoints(h3,verified_arc(point,1),verified_arc(point,2),verified_arc(point,3));
    drawnow
    xlabel('X')
    ylabel('Y')
    zlabel('Z') 
end


%% Compute the IK Look-Up Matrix

IK_matrix = [];
wayPoint_loc = [];
for i = 1:length(catching_line_matrix)
    wayPoint_loc = T_A_B * catching_line_matrix(i,:)'; % vector from the base of the robot to the target
    angleInRadians = deg2rad(-70);
    % get joint angles for each point
    IK_matrix(i,:) = inverseKineRBT(wayPoint_loc(1),wayPoint_loc(2),wayPoint_loc(3),angleInRadians);
end

save('IK_look_up_array.mat', 'IK_matrix', 'catching_line_matrix');

%% Functions

function P_mid = midpoint(P1,P2)
    P_mid(1) = (P1(1) + P2(1)) / 2;
    P_mid(2) = (P1(2) + P2(2)) / 2;
    P_mid(3) = (P1(3) + P2(3)) / 2;
end
