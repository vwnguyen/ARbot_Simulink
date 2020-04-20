% Simscape(TM) Multibody(TM) version: 7.1

% This is a model data file derived from a Simscape Multibody Import XML file using the smimport function.
% The data in this file sets the block parameter values in an imported Simscape Multibody model.
% For more information on this file, see the smimport function help page in the Simscape Multibody documentation.
% You can modify numerical values, but avoid any other changes to this file.
% Do not add code to this file. Do not edit the physical units shown in comments.

%%%VariableName:smiData


%============= RigidTransform =============%

%Initialize the RigidTransform structure array by filling in null values.
smiData.RigidTransform(1).translation = [0.0 0.0 0.0];
smiData.RigidTransform(1).angle = 0.0;
smiData.RigidTransform(1).axis = [0.0 0.0 0.0];
smiData.RigidTransform(1).ID = '';

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(1).translation = [-3.2765131551179265 -13.912021363765255 1.7541231039674103];  % in
smiData.RigidTransform(1).angle = 0;  % rad
smiData.RigidTransform(1).axis = [0 0 0];
smiData.RigidTransform(1).ID = 'RootGround[bin-1]';


%============= Solid =============%
%Center of Mass (CoM) %Moments of Inertia (MoI) %Product of Inertia (PoI)

%Initialize the Solid structure array by filling in null values.
smiData.Solid(1).mass = 0.0;
smiData.Solid(1).CoM = [0.0 0.0 0.0];
smiData.Solid(1).MoI = [0.0 0.0 0.0];
smiData.Solid(1).PoI = [0.0 0.0 0.0];
smiData.Solid(1).color = [0.0 0.0 0.0];
smiData.Solid(1).opacity = 0.0;
smiData.Solid(1).ID = '';

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(1).mass = 27.462293749627833;  % kg
smiData.Solid(1).CoM = [0 5.9221634586429266 0];  % in
smiData.Solid(1).MoI = [2819.7015260468961 2111.3901822260336 2819.7015260468975];  % kg*in^2
smiData.Solid(1).PoI = [0 0 0];  % kg*in^2
smiData.Solid(1).color = [0.792156862745098 0.81960784313725488 0.93333333333333335];
smiData.Solid(1).opacity = 1;
smiData.Solid(1).ID = 'bin*:*Default';

