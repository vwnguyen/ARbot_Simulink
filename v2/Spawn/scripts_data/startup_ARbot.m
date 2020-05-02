load('footage_v3_array.mat');
box_dim1 = [.05 .05 .05];
out_length = 0;
rotation_matrix = [4.44089209850063e-16,0.939692620785909,-0.342020143325669;-1.00000000000000,4.44089209850063e-16,0;0,0.342020143325669,0.939692620785909];

% box parameters
cube_d =.09; % 2inch cube
cube_con_rsph=.003; % radius of contact sphere block and belt

% pixels to meters
pix_2_m = 0.21875*0.0254; % pix->in->m
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
belt_l=30;
belt_h=.2;
belt_w=1.1225;
belt_spd=1.1295; % m/s
belt_in_offset=[1.4 0 0];

% camera frame parameters
camera_frame_dist = 3;  % meters away from catching line

box_angle_offset = [20 0 0]; % vector of xyz angle rotations from world to box

box_z_offset = -0.15+cube_d/2;
% vectors of xyz distance translation from world to box
box1_pos_offset = [(cube_d/2 + (belt_w - 0.606314102578163) ) (cube_d/2 + (belt_w - 0.761613922548294) ) box_z_offset]; %time 0
box2_pos_offset = [(cube_d/2 + (belt_w - 0.272408337676525) ) (cube_d/2 + (belt_w - 1.07917752695084) - (belt_spd*2.66666666666664) ) box_z_offset]; % time 2.667
box3_pos_offset = [(cube_d/2 + (belt_w - 0.617604686427116) ) (cube_d/2 + (belt_w - 0.982776597929001) - (belt_spd*7.66666666666666) ) box_z_offset]; % time 7.667
box4_pos_offset = [(cube_d/2 + (belt_w - 0.855067375230789) ) (cube_d/2 + (belt_w - 1.02983778786659) - (belt_spd*10.7333333333335)  ) box_z_offset]; % time 10.733
box5_pos_offset = [(cube_d/2 + (belt_w - 0.683449123024940) ) (cube_d/2 + (belt_w - 0.107290405601263) - (belt_spd*16.2000000000004) ) box_z_offset]; % time 16.200


shelf_pos_offset = [1.1811 1 -0.3255];
shelf_angle_offset = [90 20 90];

guard1_pos_offset = [0 0 0];
guard1_angle_offset = [270 0 90];

guard2_pos_offset = [1.1811 0 0];
guard2_angle_offset = [270 0 90];

bin_pos_offset = [1.52 1.4 .5];
bin_angle_offset = [110 180 0];

box_xyz_pos_offset = [.5588 1.15 -0.37];
box_xyz_angle_offset = [160 0 90];

% arm_pos_offset = [1.1811 1 -0.6];
% arm_angle_offset = [90 20 90];

belt_pos_offset = [1.135 -1.4 -.85];
belt_angle_offset = [180 200 -90];



% floor parameters
floor_l=3;
floor_d=.05;