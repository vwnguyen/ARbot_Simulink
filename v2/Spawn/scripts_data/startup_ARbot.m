load('bbox_array.mat');
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
belt_spd=1;
belt_in_offset=[1.4 0 0];

% camera frame parameters
camera_frame_dist = 3;  % meters away from catching line

box_angle_offset = [20 0 0]; % vector of xyz angle rotations from world to box

box_z_offset = -0.15+cube_d/2;
% vectors of xyz distance translation from world to box
box1_pos_offset = [1.146 -3.048 -1.4224];
box2_pos_offset = [1.146 -3.175 -1.4732];
box3_pos_offset = [1.146 -3.302 -1.524];
box4_pos_offset = [1.146 -3.429 -1.5748];
box5_pos_offset = [(cube_d/2 + 0.5213 ) (cube_d/2 + 0.6702 -0.2) box_z_offset];
box6_pos_offset = [0 0 0];

shelf_pos_offset = [1.7594 -1.015 -.2575];
shelf_angle_offset = [90 180 90];

guard1_pos_offset = [0.5148 .39 .192];
guard1_angle_offset = [90 160 90];

guard2_pos_offset = [1.6959 .39 .192];
guard2_angle_offset = [90 160 90];

bin_pos_offset = [2.1 -1.25 -1.25];
bin_angle_offset = [90 0 0];

arm_pos_offset = [1.14 -1.15 -.2575];
arm_angle_offset = [0 0 90];

belt_pos_offset = [1.135 -1.4 -.85];
belt_angle_offset = [180 200 -90];



% floor parameters
floor_l=3;
floor_d=.05;