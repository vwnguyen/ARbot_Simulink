%% Environment Parameters
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

belt_spd = 1;

%% Suction Parameters
attraction_stiffness = -1e7; %-1e5
attraction_damp = 0.001; % 10
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
max_Catching_Time = 0.5; % 0.5, 1, 2
picks_per_minute = (1/max_Catching_Time) * 60;
resolution_of_trajectory = 50;
sampleTime = max_Catching_Time / resolution_of_trajectory;
queue_ISR_sample_time = 0.1;

% default belt speed 1.1295
% belt_spd = 2;
inherentCPUCost = 0.72;

% max catch time = 2
% queue_ISR_sample_time = 0.1;
% 30 PPM CPU COSTS
% load in belt_spd
% belt spds 0.5 -> 1.5 
% inherentCPUCOST = 0.7
% belt spds 1.75 -> 2
% inherentCPUCOST = 0.15

% max catch time = 1
% queue_ISR_sample_time = 0.1;
% 60 PPM CPU COSTS
% load in belt_spd
% belt spds 0.5 
% inherentCPUCOST = 0.7
% belt spds 1
% inherentCPUCOST = 0.72
% belt spds 1.25
% inherentCPUCOST = 0.6
% belt spds 1.5
% inherentCPUCOST = 0.4
% belt spds 1.75
% inherentCPUCOST = 0.2
% belt spds 2
% inherentCPUCOST = 0

% max catch time = 0.5
% queue_ISR_sample_time = 0.1;
% 120 PPM CPU COSTS
% belt spds 0.5 
% inherentCPUCOST = 0.72
% belt spds 0.75
% inherentCPUCOST = 0.815
% belt spds 1
% inherentCPUCOST = 0.68
% belt spds 1.25
% inherentCPUCOST = 0.68
% belt spds 1.5 
% inherentCPUCOST = 0.36
% belt spds 1.75 
% inherentCPUCOST = 0.15
% belt spds 2 
% inherentCPUCOST = 0

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
             
numTargets = length(P_C);

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
eeOrientation = deg2rad(-70);
initial_joints = [deg2rad(80) 0 0 0];
deposit_joint_angle = initial_joints(1) - 0.00025;

% % uncomment to use the catching arc iterative
[P_B  distanceToCatchLine timeToCatchLine ikSol P_C] = ...
mapToCatchArcIterative(P_B_CORG,P_C,belt_spd,robot_base_to_camera_frame_rot, ...
max_Catching_Time,camera_frame_dist,workspace_points,ikLookup,...
d1_function_coeff,t1_function_coeff);

testing_array = timeToCatchLine - max_Catching_Time/2 + inherentCPUCost

% if the arm CANNOT attain those targets, append to testing_array, and
% ikSols, to accomodate stateflow 

if length(testing_array) < numTargets 
    for i = length(testing_array):(numTargets-1)
        testing_array = [ testing_array 100 ]; % append an impossible time
        % ikSol = [ ikSol; initial_joints]; % iksol already been sovled for
    end
end