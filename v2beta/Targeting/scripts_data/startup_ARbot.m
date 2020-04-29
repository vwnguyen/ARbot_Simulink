%{
Variables needed by the ARbot simulink file
%}
% joint configurations
% q0=[0 pi/4 pi/4 pi/2];                   % start point
q0=[0 0 0 0]; 
% transformation to catching line information
rotation_matrix = [4.44089209850063e-16,0.939692620785909,-0.342020143325669;-1.00000000000000,4.44089209850063e-16,0;0,0.342020143325669,0.939692620785909];
T_A_B = [ 0 0 0 0;
  0 0 0 0;
  0 0 0 0;
  0 0 0 1;
  ];
P_A_BORG = [[0.559474096408469];[0.563879950000000];[-0.184156331568441]];
P_B = [0;0;0;1;];
% P_A = rotation_matrix * P_A_BORG + P_B;

T_A_B(1:3,1:3) = rotation_matrix;
T_A_B(1:3,4) = P_A_BORG;
P_A = T_A_B * P_B;

angleInRadians = deg2rad(-90);
th = inverseKineRBT(P_A(1),P_A(2),P_A(3),angleInRadians)

qv=[0 -pi/8 pi/8 pi/8]; % mid point 
% qv=[0 0 0 0];
qf=[0 0 0 0];       % end point
% 
% q0 = [-pi/2 pi/4 -pi/4 -pi/2];
% qv=[.5   -0.5    0   0]; % mid point
% qf = [-pi/2 pi/4 -pi/4 pi/2];

th=traj6_v2(q0,th,qf,2,1);                  % generate the trajectory
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