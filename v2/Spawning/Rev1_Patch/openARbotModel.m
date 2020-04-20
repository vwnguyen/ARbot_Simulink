%% Load and display robot
clear all
close all

% set the path variable to where you have saved ARbot_v1_4Export_5 folder
path='';
addpath(path);
robot=importrobot('ARbot_v1_4Export_5.urdf');

% Add gravity
gravityVec = [0 0 -9.80665];
robot.Gravity = gravityVec; 
axes=show(robot); 

%%        
% Add another massless coordinate frame for the end effector
eeOffset = [.256 .052 0];
eeBody = robotics.RigidBody('end_effector');
eeBody.Mass = 0;
eeBody.Inertia = [0 0 0 0 0 0];
setFixedTransform(eeBody.Joint,trvec2tform(eeOffset));
figure(1);
addBody(robot,eeBody,'L4');
axes=show(robot); 
%showdetails(robot);



