%{
Variables needed by the ARbot simulink file
%}
%{
% joint configurations
%q0=[0 pi/4 pi/4 pi/4];                    % start point
q0=[0	0	0	0]*(pi/180);
qv=[90	90	0	0]*(pi/180);
qf=[-90	0	0	0]*(pi/180);       % end point

th=traj6_v2(q0,qv,qf,2,1); % generate the trajectory
% conveyor belt parameters
belt_l=2; %m
belt_h=.02;
belt_w=1.24; % about 20 inches
belt_spd=0.1;
belt_in_offset=[.7 0 0];

% box parameters
cube_d=.09; % 
cube_con_rsph=.003; % radius of contact sphere block and belt


% floor parameters
floor_l=3;
floor_d=.05;

%L1=.227m
%L2=.317m
%L3=.343m
%L4=.150m
%}

% Modify for your own path %
addpath('description\urdf');
robot=importrobot('arbot_betav1.urdf');

% Add gravity
gravityVec = [0 0 -9.80665];
robot.Gravity = gravityVec; 

% Add another massless coordinate frame for the end effector
eeOffset = 0.155;
eeBody = robotics.RigidBody('end_effector');
eeBody.Mass = 0;
eeBody.Inertia = [0 0 0 0 0 0];
setFixedTransform(eeBody.Joint,trvec2tform([eeOffset 0 0]));
figure(1);
addBody(robot,eeBody,'L4');
axes=show(robot); 
