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


% two seconds to complete trajectory
% second to arrive at via-point location
th=traj6_v2(q0,qv,qf,2,1);                  % generate the trajectory
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