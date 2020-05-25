%% Load in parameters
load('user_inputs.mat');

%% Environment Parameters
rotation_matrix = [4.44089209850063e-16,0.939692620785909,-0.342020143325669;-1.00000000000000,4.44089209850063e-16,0;0,0.342020143325669,0.939692620785909];

% box parameters
box1_dim = [box_dim(1,1) box_dim(1,2) 0.0254/2];
box2_dim = [box_dim(2,1) box_dim(2,2) 0.0254/2];
box3_dim = [box_dim(3,1) box_dim(3,2) 0.0254/2];
box4_dim = [box_dim(4,1) box_dim(4,2) 0.0254/2];
box5_dim = [box_dim(5,1) box_dim(5,2) 0.0254/2];
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
%% Suction Parameters
attraction_stiffness = -1e5; %-1e5
attraction_damp = 10; % 10
frame_sep = 0.05; %0.05
deadzone = 1; %1

%% Conveyor Belt Parameters
belt_l=30;
belt_h=.2;
belt_w=1.1225;
belt_in_offset=[1.4 0 0];

catch_line_dist = belt_l * 9.65/10; % distance from the bot belt to catch
camera_frame_dist = 3; % distance from the camera frame to catch line
                                                  
%% box parameters
box_angle_offset = [20 0 0]; % vector of xyz angle rotations from world to box
% box_z_offset = -0.15+cube_d/2;
box_z_offset = -0.1547 + inch/2;
% 0.0254 is an inch

box_positions = correct_positions(box_positions, box1_dim, box2_dim, box3_dim, box4_dim, box5_dim);
% position vectors of the box with reference to the camera frame
box1_pos_offset = [(box1_dim(1)/2 + box_positions(1,1)) (box1_dim(2)/2 - box_positions(1,2))  box_z_offset] 
box2_pos_offset = [(box2_dim(1)/2 + box_positions(2,1)) (box2_dim(2)/2 - box_positions(2,2))  box_z_offset] 
box3_pos_offset = [(box3_dim(1)/2 + box_positions(3,1)) (box3_dim(2)/2 - box_positions(3,2))  box_z_offset] 
box4_pos_offset = [(box4_dim(1)/2 + box_positions(4,1)) (box4_dim(2)/2 - box_positions(4,2))  box_z_offset] 
box5_pos_offset = [(box5_dim(1)/2 + box_positions(5,1)) (box5_dim(2)/2 - box_positions(5,2))  box_z_offset] 


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
              
dynamic_offset_parameters = load('fitted_coefficients.mat');
distances = dynamic_offset_parameters.distances; 
acc_times = dynamic_offset_parameters.acc_times; 
spds = dynamic_offset_parameters.spds; 
% plot(spds,distances);
% hold on          
d1_function_coeff = dynamic_offset_parameters.d1_function_coeff;
t1_function_coeff = dynamic_offset_parameters.t1_function_coeff;

% uncomment for workspace with offset and perpendicular
workspace = load('ARbot_workspace_iterative_P_A_const.mat');
ikSols = load('ikLookupiterativeP_A_const.mat');
workspace_points = workspace.P_A;
sizeOfWorkspace = size(workspace_points);
if sizeOfWorkspace(2) > 3
    workspace_points(:,4) = [];
end
ikLookup = ikSols.ikLookup;

% safe via point for intermediate trajectories
safe_via_angles = [ 2.0027    0.9535    1.3824    0.7694] ;
J1_safe_angle = rad2deg(safe_via_angles(1)) - 90;
J1_safe_angle = deg2rad(J1_safe_angle);
safe_via_angles(1) = J1_safe_angle;

% % test catching arc from camera frame

%% Map targets and calculate ikSolutions
% testing_array = [ 1.25 5 8 11];
eeOrientation = deg2rad(-70);
max_Catching_Time = 2;

initial_joints = [deg2rad(85) pi/4 pi/4 pi/4];
deposit_joint_angle = initial_joints(1) - 0.00025;

% uncomment to use the catching arc iterative
[P_B  distanceToCatchLine timeToCatchLine ikSol P_C] = ...
mapToCatchArcIterative(P_B_CORG,P_C,belt_spd,robot_base_to_camera_frame_rot, ...
max_Catching_Time,eeOrientation,camera_frame_dist,workspace_points,ikLookup,...
d1_function_coeff,t1_function_coeff);

testing_array = timeToCatchLine - max_Catching_Time/2 + inherentCPUCost;

% testing_array = [ 2.68 5.27 10.19 13.3 18.02 ] - max_Catching_Time/2 - inherent_Time_Delay; 

% ikSol = inverseKineRBT(P_B(1,1),P_B(1,2),P_B(1,3),eeOrientation)

%% Functions

% correct_positions(): depending on the position that the box is placed at,
% we always want the box within the belt. Because the centroids are in the
% middle of the boxes, the position range is from 0 to belt_w-box_dim/2. Any
% more than that and the box is in the gaurd or sticking out the belt. This
% function will correct for this corner case
function final_positions = correct_positions(positions, box1, box2, box3, box4, box5)
    width = 1.1225; % m
    if positions(1,1) > (width - box1(1)/2)
        positions(1,1) = positions(1,1)-box1(1);
    end
    if positions(2,1) > (width - box2(1)/2)
        positions(2,1) = positions(2,1)-box2(1);
    end
    if positions(3,1) > (width - box3(1)/2)
        positions(3,1) = positions(3,1)-box3(1);
    end
    if positions(4,1) > (width - box4(1)/2)
        positions(4,1) = positions(4,1)-box4(1);
    end
    if positions(5,1) > (width - box5(1)/2)
        positions(5,1) = positions(5,1)-box5(1);
    end
    final_positions = positions;
end