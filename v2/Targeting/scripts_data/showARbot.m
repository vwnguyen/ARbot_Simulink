%% add urdf file to path
%addpath('C:\Users\logan\Documents\MATLAB\Robotics\Arbot_v1\description\urdf');
rbot=importrobot('arbot_v1.urdf');
%% add end effector frame
eeOffset = 0.3;
eeBody = robotics.RigidBody('end_effector');
eeBody.Mass = 0;
eeBody.Inertia = [0 0 0 0 0 0];
setFixedTransform(eeBody.Joint,trvec2tform([eeOffset 0 0]));
addBody(rbot,eeBody,'L4');
figure(2);
axes=show(rbot); 

