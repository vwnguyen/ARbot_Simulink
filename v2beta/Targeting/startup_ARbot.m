%{
Variables needed by the ARbot simulink file
%}
% conveyor belt parameters
belt_l=3.50;
belt_h=.2;
belt_w=1.1225;
belt_spd=1;
belt_in_offset=[1.4 0 0];

catch_line_dist = belt_l*(3/4);

% box parameters
cube_d=.09; % 2inch cube
cube_con_rsph=.003; % radius of contact sphere block and belt

% floor parameters
floor_l=3;
floor_d=.05;
%% URDF Model path and loading

% add path of urdf model
%addpath('C:\Users\Viet\Desktop\git_repos\ARbot_Simulink\v2beta\Environment\PlantModel');
environment = 'C:\Users\Viet\Desktop\git_repos\ARbot_Simulink\v2beta\Environment\PlantModel'
addpath(genpath(environment));

% add the end effector
robot=importrobot('arbot_betav1.urdf');
% Add gravity
gravityVec = [0 0 -9.80665];
robot.Gravity = gravityVec; 
% Add another massless coordinate frame for the end effector
eeOffset = 0.155;
eeBody = robotics.RigidBody('end_effector');
eeBody.Mass = 0;
eeBody.Inertia = [0 0 0 0 0 0];
setFixedTransform(eeBody.Joint,trvec2tform([eeOffset 0 0]));
% figure(1);
addBody(robot,eeBody,'L4');
%axes=show(robot); 
showdetails(robot);
%% Trajectory and Ikine stuff
base_to_catch_out = [  0.0000    0.9397   -0.3420;
   -1.0000    0.0000         0;
         0    0.3420    0.9397];

rotation_matrix = base_to_catch_out;
     
T_A_B = [ 0 0 0 0;
  0 0 0 0;
  0 0 0 0;
  0 0 0 1;
  ];

P_A_BORG = [[0.5595];[0.5639];[  -0.1842]];

% where at the catching line it should end effector go 
P_B = [ belt_w / 2; 0; 0; 1;];

T_A_B(1:3,1:3) = base_to_catch_out;
T_A_B(1:3,4) = P_A_BORG;
P_A = T_A_B * P_B; % vector from the base of the robot to the target
%% Test Array
testing_array = [ 1.25 5 8 11]; % when to get the target

angleInRadians = deg2rad(-70);
ikSol = inverseKineRBT(P_A(1) , P_A(2) ,P_A(3),angleInRadians);

q0=[-pi/2 0 0 0];
qv=[0 -pi/8 pi/8 pi/8]; % mid point 
% qv=[0 0 0 0];
qf=[-pi/2 0 0 0];       % end point
% 
% q0 = [-pi/2 pi/4 -pi/4 -pi/2];
% qv=[.5   -0.5    0   0]; % mid point
% qf = [-pi/2 pi/4 -pi/4 pi/2];

th=traj6_v2(q0,ikSol,qf,2,1);                  % generate the trajectory