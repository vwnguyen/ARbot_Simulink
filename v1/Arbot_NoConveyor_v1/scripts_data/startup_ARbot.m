%{
Variables needed by the ARbot simulink file
%}
% create ARbot with defauly values
% ARbot = exampleHelperARbotPickPlace;


% joint configurations
q0=[0 pi/4 -pi/4 -pi/4];                    % start point
qv=[.5   -0.2385    1.1573   -2.4896]; % mid point 
qf = [-pi/2 pi/4 -pi/4 -pi/2]; % some generic bin location 
testing_array = [ 1 3 6 9 ];
temp = load('facilityIDs_30FPS_30s.mat');
facility_data = temp.filtered_IDs;
sample_time_30FPS = 1/30;

rotation_matrix =[ 0     0     1;
                  -1     0     0;
                   0    -1     0];
T_A_B = [ 0 0 0 0;
  0 0 0 0;
  0 0 0 0;
  0 0 0 1;
  ];
% at the reference frame
P_B = [0;0;0;1;];
% rotations from frame to frame
T_A_B(1:3,1:3) = rotation_matrix;
% displacement from frame to frame
P_A_BORG = [ 0.45; 0; 0.01];
T_A_B(1:3,4) = P_A_BORG;
P_A = T_A_B * P_B;
angleInRadians = deg2rad(-90);
ikSol = inverseKineRBT(P_A(1),P_A(2),P_A(3),angleInRadians)               
               
% two seconds to complete trajectory
% second to arrive at via-point location
% ikSol = inverseKineRBT();
th=traj6_v2(q0,ikSol,qf,2,1);                  % generate the trajectory
% conveyor belt parameters
belt_l=2; %m
belt_h=.02;
belt_w=.5; % about 20 inches
belt_spd=0.1;
belt_in_offset=[.7 0 0];

% box parameters
cube_d=.09; % 2inch cube
cube_con_rsph=.003; % radius of contact sphere block and belt

% floor parameters
floor_l=3;
floor_d=.05;