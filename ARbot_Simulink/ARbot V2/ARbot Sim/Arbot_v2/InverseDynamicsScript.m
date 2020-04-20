addpath('C:\Users\logan\Documents\MATLAB\Robotics\dynamic_sim_v1\4dof_v12\urdf');
robot=importrobot('4dof_v12.urdf');
axes=show(robot); 
config = homeConfiguration(robot);

%% generate a trajectory
%t0=0; % initial time
%tf=1; % total time for trajectory
%tv=tf/2; % time for via to be reached
% joint configurations
%q0=[0 pi/4 -pi/4 -pi/4];                    % start point
%qv=[-0.2809   -0.2385    1.1573   -2.4896]; % mid point
%qv=[0 0 0 0];
%qf=[3.14 -0.2385    1.1573   -2.4896];      % end point
%qf=q0;
[th, th_dot, th_ddot]=traj6_v2(q0,qv,qf,2,1);                  % generate the trajectory


robot.DataFormat = 'row';
robot.Gravity = [0 0 -9.81];

%% inverse dynamics
for i=1:length(th)
    tau2(i,:) = inverseDynamics(robot, [th(i,1) th(i,2) th(i,3) th(i,4)], [th_dot(i,1) th_dot(i,2) th_dot(i,3) th_dot(i,4)],[th_ddot(i,1) th_ddot(i,2) th_ddot(i,3) th_ddot(i,4)]);
end

%plot(tau2();
%plot(th_dot);
max(tau2(:,1))
min(tau2(:,1))
median(tau2(:,1))
mean(tau2(:,1))
rms(tau2(:,1))