%% Environment Parameters

load('footage_v3_array.mat');
box_dim1 = [.05 .05 .05];
out_length = 0;
rotation_matrix = [4.44089209850063e-16,0.939692620785909,-0.342020143325669;-1.00000000000000,4.44089209850063e-16,0;0,0.342020143325669,0.939692620785909];

% box parameters
box1_dim = [0.170036163091660 0.182921404933929 0.0254/2];
box2_dim = [0.184457923913002 0.172139129447937 0.0254/2];
box3_dim = [0.055932275152206 0.103630726337433 0.0254/2];
box4_dim = [0.110056496906281 0.147900141429901 0.0254/2];
box5_dim = [0.121905452680588 0.165243265843391 0.0254/2];
cube_d =.09; % 2inch cube
cube_con_rsph=.003; % radius of contact sphere block and belt

% floor parameters
floor_l=3;
floor_d=.05;

% pixels to meters
pix_2_m = 0.21875*0.0254; % pix->in->m
inch = 0.0254;
%{
Variables needed by the ARbot simulink file
%}

%% Conveyor Belt Parameters
belt_l=30;
belt_h=.2;
belt_w=1.1225;
belt_spd=1.1295; % m/s
belt_in_offset=[1.4 0 0];

catch_line_dist = belt_l * 9.65/10; % distance from the bot belt to catch
camera_frame_dist = 3; % distance from the camera frame to catch line
                                                  
%% box parameters
box_angle_offset = [20 0 0]; % vector of xyz angle rotations from world to box
% box_z_offset = -0.15+cube_d/2;
box_z_offset = -0.1547 + inch/2;
% 0.0254 is an inch

% position vectors of the box with reference to the camera frame
box1_pos_offset = [(box1_dim(1)/2 + (belt_w - 0.606314102578163) ) (box1_dim(2)/2 + (belt_w - 0.761613922548294) ) box_z_offset ]; %time 0
box2_pos_offset = [(box2_dim(1)/2 + (belt_w - 0.272408337676525) ) (box2_dim(2)/2 + (belt_w - 1.07917752695084 + 0.4226) - (belt_spd*(2.66666666666664)) ) box_z_offset ]; % time 2.667
box3_pos_offset = [(box3_dim(1)/2 + (belt_w - 0.617604686427116) ) (box3_dim(2)/2 + (belt_w - 0.982776597929001 + 0.4572) - (belt_spd*(7.66666666666666)) ) box_z_offset ]; % time 7.667
box4_pos_offset = [(box4_dim(1)/2 + (belt_w - 0.855067375230789) ) (box4_dim(2)/2 + (belt_w - 1.02983778786659 + 0.4353) - (belt_spd*(10.7333333333335))) box_z_offset ]; % time 10.733
box5_pos_offset = [(box5_dim(1)/2 + (belt_w - 0.683449123024940) ) (box5_dim(2)/2 + (belt_w - 0.107290405601263 + 0.4270) - (belt_spd*(16.2000000000004)) ) box_z_offset ]; % time 16.200

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

chute_pos_offset = [1.21285 .7 .1];
chute_angle_offset = [70 180 180];

belt_pos_offset = [1.135 -1.4 -.85];
belt_angle_offset = [180 200 -90];
%% Trajectory Parameters
% camera_frame_dist is the distance from the camera
P_B_CORG = [    -2.5986;     0.5588;     -1.3395;];
P_B_Catch = [ 0.2205; 0.5588; -0.3135];
robot_base_to_camera_frame_rot = [   0.0000    0.9397   -0.3420;
                                   -1.0000    0.0000         0;
                                         0    0.3420    0.9397;];
P_C =           [box1_pos_offset; 
                 box2_pos_offset; 
                 box3_pos_offset;
                 box4_pos_offset;
                 box5_pos_offset];
             

P_C(:,3) = inch/2;  % z axis a half inch above the belt
P_C(:,4) = 1; % append a column of ones for matrix transformation 
              % calculations

% filtered variables

% workspace_filtered = load('ARbot_workspace_filtered.mat');
% workspace_points = workspace_filtered.filtered_workspace;
% workspace_ind = floor(length(workspace_points));
% Arc_Midpoint = workspace_points(floor(workspace_ind/2),:);
% Arc_Start =  workspace_points(1,:);
% Arc_End =  workspace_points(workspace_ind,:);


% corrected variables
workspace = load('ARbot_workspace.mat');
workspace_points = workspace.corrected_workspace;
workspace_points(:,4) = [];
ikSols = [];
workspace_ind = floor(length(workspace_points));
Arc_Midpoint = workspace_points(floor(workspace_ind/2),:);
Arc_Start = workspace_points(1,:);
Arc_End =  workspace_points(workspace_ind,:);

% % test catching arc from camera frame

%% Map targets and calculate ikSolutions
% testing_array = [ 1.25 5 8 11];
eeOrientation = deg2rad(-70);
max_Catching_Time = 2;

% uncomment to use the catching arc
[P_B  distanceToCatchLine timeToCatchLine ikSol P_C] = ...
mapToCatchArc(P_B_CORG,P_C,belt_spd,robot_base_to_camera_frame_rot, ...
max_Catching_Time,eeOrientation,camera_frame_dist,workspace_points,ikSols);

% uncomment to use the catching line
% [P_B  distanceToCatchLine timeToCatchLine ikSol P_C] = ...
% mapToCatchLineSim(P_B_CORG,P_C,belt_spd,robot_base_to_camera_frame_rot, ...
% max_Catching_Time,eeOrientation,camera_frame_dist);

% testing_array =[ 0 2.6670 7.6670 10.7330 16.2000];  % when targets are spotted at the camera frame
% time to execute the trajectories
testing_array = timeToCatchLine - max_Catching_Time/2;
inherent_Time_Delay = 0.2;

% testing_array = [ 2.68 5.27 10.19 13.3 18.02 ] - max_Catching_Time/2 - inherent_Time_Delay; 

% ikSol = inverseKineRBT(P_B(1,1),P_B(1,2),P_B(1,3),eeOrientation)
