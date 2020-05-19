% Usage: change your current working directory to the v2offset folder
% then run this script, which adds all necessary paths.

% get current working directory
cwd = pwd;
% append necessary folder strings to cwd
Environment = append(cwd,'\Environment');
Workspace = append(cwd,'\workspace_points');
Scripts_Data = append(cwd,'\scripts_data');

addpath(genpath(Environment));
addpath(genpath(Workspace));
addpath(genpath(Scripts_Data));
