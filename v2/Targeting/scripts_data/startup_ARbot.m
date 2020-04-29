%{
Variables needed by the ARbot simulink file
%}
% conveyor belt parameters
belt_l=3.50;
belt_h=.2;
belt_w=1.1225;
belt_spd=1;
belt_in_offset=[1.4 0 0];

% box parameters
cube_d=.09; % 2inch cube
cube_con_rsph=.003; % radius of contact sphere block and belt

% floor parameters
floor_l=3;
floor_d=.05;

%% Trajectory and Ikine stuff

% add path of urdf model
addpath('C:\Users\Viet\Desktop\git_repos\ARbot_Simulink\v2\Environment\description\urdf');
rbot=importrobot('arbot_v1_Beta.urdf');
showdetails(rbot);

% joint configurations and inverse kine stuff
 
% transformation to catching line information
rotation_matrix = [4.44089209850063e-16,0.939692620785909,-0.342020143325669;
    -1.00000000000000,4.44089209850063e-16,0;
    0,0.342020143325669,0.939692620785909];

base_to_catch_out = [ 0.0000    0.9397   -0.3420;
   -1.0000    0.0000         0;
         0    0.3420    0.9397];


T_A_B = [ 0 0 0 0;
  0 0 0 0;
  0 0 0 0;
  0 0 0 1;
  ];

P_A_BORG = [[0.559474096408469];[0.563879950000000];[-0.184156331568441]];

% where the middle of the belt is 
P_B = [ belt_w / 2 ; 0; cube_d; 1;];

T_A_B(1:3,1:3) = base_to_catch_out;
T_A_B(1:3,4) = P_A_BORG;
P_A = T_A_B * P_B;

testing_array = [ 2.4 6 9 12];

angleInRadians = deg2rad(-70);
ikSol = inverseKineRBT(P_A(1) , 0 ,P_A(3),angleInRadians)

q0=[-pi/2 0 0 0];
qv=[0 -pi/8 pi/8 pi/8]; % mid point 
% qv=[0 0 0 0];
qf=[-pi/2 0 0 0];       % end point
% 
% q0 = [-pi/2 pi/4 -pi/4 -pi/2];
% qv=[.5   -0.5    0   0]; % mid point
% qf = [-pi/2 pi/4 -pi/4 pi/2];

th=traj6_v2(q0,ikSol,qf,2,1);                  % generate the trajectory