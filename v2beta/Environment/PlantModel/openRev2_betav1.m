%% Load and display robot
clear all
close all

%L1=.227m
%L2=.317m
%L3=.343m
%L4=.150m

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

%%
% Generates a simulink file for the Rev2 beta

%smimport('arbot_betav1.urdf');

%%
% from J2 to the origin of J4 the robot is .660m (26 inches)
% J2 is the same distance off the ground as the previous version (.2271m)
