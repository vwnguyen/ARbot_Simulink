% %{
% Here I'll attempt to calculate the forward HTM for the new design revision
% of the robot arm.
% 
% i    alpha_i    a_i    d_i    th_i
% ----------------------------------
% 1  |  90     |   0   | .2271 | th_1
% 2  |  0      | .4011 | .1016 | th_2
% 3  |  0      | .382  |   0   | th_3
% 4  |  0      | .256  |   0   | th_4
% %}


% add path of urdf model
% import the arbotv2 model
addpath('C:\Users\Viet\Desktop\git_repos\ARbot_Simulink\v2\Environment\description\urdf');

% import the arbot v1 model
% addpath('C:\Users\Viet\Desktop\git_repos\ARbot_Simulink\v1\Arbot_NoConveyor_v1\description\urdf');
robot=importrobot('arbot_v1_Beta.urdf');

% import the arbotv1 model

eeOffsetX = 0.256;
eeBody = robotics.RigidBody('end_effector');
setFixedTransform(eeBody.Joint,trvec2tform([eeOffsetX 0 0]));
addBody(robot,eeBody,'L4');
show(robot);

%% CLOSED LOOP INVERSE KINEMATICS 

% calculate the joint angles ( in radians ) 
% general test
%jointAngles = inverseKineRBT(0.5,0.5,0,-pi/2);
% catch line corner left
rads = deg2rad(-70);
% jointAngles = inverseKineRBT(P_A(1),P_A(2),P_A(3),rads);
% catch line corner right
% jointAngles = inverseKineRBT(-P_A(1),P_A(2),P_A(3),rads);
% middle of the catch line 
jointAngles = inverseKineRBT(P_A(1),0,P_A(3),rads);

%% show the robot in new config
newPose = robot.randomConfiguration;
newPose(1).JointPosition = jointAngles(1);
newPose(2).JointPosition = jointAngles(2);
newPose(3).JointPosition = jointAngles(3);
newPose(4).JointPosition = jointAngles(4);
show(robot,newPose);

%% MATLAB INVERSE KINEMATICS SECTION
ik = inverseKinematics('RigidBodyTree',robot);
% Use a lower magnitude weight for the orientation angles than the 
% position components. Use the home configuration of the robot as an 
% initial guess.
weights = [0.25 0.25 0.25 1 1 1];
% start at 0 0 0 0
initialguess = robot.homeConfiguration;
%% Test different joint angles to verify the DH params

newPose = robot.randomConfiguration;
newPose(1).JointPosition = 0;
newPose(2).JointPosition = 0;
newPose(3).JointPosition = 0;
newPose(4).JointPosition = pi/4;

show(robot,newPose);

%%
% obtrain the transformation matrix from base to end effector
% newPose = [ 1 0 0 0;
%             0 1 0 0;
%             0 0 1 0;
%             0 0 0 1];
        
tform2 = getTransform(robot,newPose,'end_effector');
% [ee_pos tform3] = forwardKineRev_1(newPose(1).JointPosition,newPose(2).JointPosition,newPose(3).JointPosition ,newPose(4).JointPosition) 

         
% tform3(1:3,4) = [ 0,0,0];
x_ang = 0;
y_ang = 0;
z_ang = -90;
RX = rotx(x_ang);
RY = roty(y_ang);
RZ = rotz(z_ang);
rotmat = RX*RY*RZ;

% tform2(1:3,1:3) = [ 1 0 0;
%             0 1 0;
%             0 0 1];

tform2(1:3,1:3) = rotmat;
        
tform2(1:3,4) = [ 0.5,0.5,0];

% tform3(1:3,1:3) = rotmat;

%%
weights = [0.25 0.25 0.25 1 1 1];

% start at 0 0 0 0
initialguess = robot.homeConfiguration;

% use that transformation matrix, along with weights and initial guess to 
% obtain that new pose.

[configSoln2,solnInfo2] = ik('end_effector',tform2,weights,initialguess);
show(robot,configSoln2);

%% helper functions
function rot_x = rotx(ang)

rot_x = [
  1 0 0;
  0 cosd(ang) -sind(ang);
  0 sind(ang) cosd(ang) ;
];

end

function rot_y = roty(ang)

rot_y = [
    cosd(ang) 0 sind(ang);
    0 1 0;
    -sind(ang) 0 cosd(ang);
];

end

function rot_z = rotz(ang)

rot_z = [
  cosd(ang) -sind(ang) 0;
  sind(ang) cosd(ang) 0;
  0 0 1;
];

end


