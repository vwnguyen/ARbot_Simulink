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

% box parameters
cube_d=.09; % 2inch cube
cube_con_rsph=.003; % radius of contact sphere block and belt

% floor parameters
floor_l=3;
floor_d=.05;