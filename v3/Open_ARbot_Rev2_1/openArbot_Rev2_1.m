%% Load and display robot
clear all
close all

% Modify for your own path %
% parent = '..\Targeting';
% environment = '..\Environment';
% addpath(genpath(parent));
% addpath(genpath(environment));

robot=importrobot('Arbot_Rev2_1.urdf');

% Add gravity
gravityVec = [0 0 -9.80665];
robot.Gravity = gravityVec; 

% Add another massless coordinate frame for the end effector

eeOffset = 0.150;
eeBody = robotics.RigidBody('end_effector');
eeBody.Mass = 0;
eeBody.Inertia = [0 0 0 0 0 0];
setFixedTransform(eeBody.Joint,trvec2tform([eeOffset 0 0]));
figure(1);
addBody(robot,eeBody,'L4');
pose = robot.homeConfiguration;
pose(1).JointPosition = pi/4;
pose(2).JointPosition = pi/4;
pose(3).JointPosition = pi/4;
pose(4).JointPosition = pi/4;
grid off
show(robot,pose,'PreservePlot', true,'Frames','off');
grid off
%smimport('Arbot_Rev2_1.urdf');

%%
% from J2 to the origin of J4 the robot is .660m (26 inches)
% J2 is the same distance off the ground as the previous version (.2271m)
