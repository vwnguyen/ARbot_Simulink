load('footage_array.mat');
box_dim1 = [.05 .05 .05];
out_length = 0;
rotation_matrix = [4.44089209850063e-16,0.939692620785909,-0.342020143325669;-1.00000000000000,4.44089209850063e-16,0;0,0.342020143325669,0.939692620785909];

%{
Variables needed by the ARbot simulink file
%}
% joint configurations
q0=[0 pi/4 pi/4 pi/4];                   % start point
qv=[0 -pi/8 pi/8 pi/8]; % mid point 
% qv=[0 0 0 0];
qf=[0 pi/4 (2*pi)/3 (11*pi)/18];       % end point

% 
% q0 = [-pi/2 pi/4 -pi/4 -pi/2];
% qv=[.5   -0.5    0   0]; % mid point
% qf = [-pi/2 pi/4 -pi/4 pi/2];

th=traj6_v2(q0,qv,qf,2,1);                  % generate the trajectory
% conveyor belt parameters
belt_l=3.50;
belt_h=.2;
belt_w=1.1225;
belt_spd=1;
belt_in_offset=[1.4 0 0];


box1_pos_offset = [1.146 -3.048 -1.4224];
box1_angle_offset = [20 0 0];

box2_pos_offset = [1.146 -3.175 -1.4732];
box2_angle_offset = [20 0 0];

arm_pos_offset = [1.14 -1.15 -.2575];
arm_angle_offset = [0 0 90];

box3_pos_offset = [0.629 -3.034 -1.275];
box3_angle_offset = [20 0 0];

% box parameters
cube_d=.09; % 2inch cube
cube_con_rsph=.003; % radius of contact sphere block and belt

% floor parameters
floor_l=3;
floor_d=.05;