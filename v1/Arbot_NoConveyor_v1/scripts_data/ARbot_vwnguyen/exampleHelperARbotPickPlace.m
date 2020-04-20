classdef exampleHelperARbotPickPlace < handle
% This class is for internal use and may be removed in a future release
%
%exampleHelperCoordinatorPickPlaceROSGazebo Class used to run the Stateflow
%chart and setup ROS communication with Gazebo
%   This class is used to control the pick-and-place workflow execution.
%   The class serves two main purposes:
%      1. It holds information about ROS-MATLAB communication (subscribers,
%      publishers etc.)
%      2. It stores all data about the current pick and place job
%
% Copyright 2020 The MathWorks, Inc.

    properties         
        FlowChart
        Robot
        World = {};
        Parts = {};
        Obstacles = {};
        ObstaclesVisible = {};
        DetectedParts = {};
        RobotEndEffector
        CurrentRobotJConfig
        CurrentRobotTaskConfig
        NextPart = 0;
        PartOnRobot = 0;
        HomeRobotTaskConfig 
        PlacingPose
        GraspPose
        Figure
        TimeStep
        MotionModel
        NumJoints
        NumDetectionRuns = 0;
        CollisionHelper
        ROSinfo
        
        
        InitPose
        TrajectoriesExecuted
        TrajectoriesSpeed
        ViaPointTime
        DetectorModel
        Bboxes
        Labels
        img
    end
    
    methods
        
        % Constructor, creates and initializes any info needed
        % begins with default values
        function obj = exampleHelperARbotPickPlace()
            temp = load('tinyYOLO_03.mat');
            obj.TrajectoriesExecuted = 0;
            obj.TrajectoriesSpeed = 2;
            obj.ViaPointTime = obj.TrajectoriesSpeed/2;
            obj.InitPose = [0 pi/4 -pi/4 -pi/4]; 
            obj.DetectorModel = temp.detector;                     
        end
        
        % assumes all resolutions are the same size
        % Params:
        % obj = original obj 
        % newImg = new image in matrix format 
        function changed = changeImg(obj,newImg)
            % Load deep learning model for object detection
            % check if theres any change 
            if (isempty(obj))
                disp('Error: obj is empty');
                return;
            end
            
            % check if obj.img is empty first, for dimensionality
            if (isempty(obj.img))
                obj.img = newImg;
                changed = 1;
                return;
            end
            
            if (obj.img ~= newImg )
                changed = 1;
                obj.img = newImg;
                return;
            end
            changed = 0;
        end
        
        function isMoving = getMovementStatus(obj)
%             statusMsg = receive(obj.ROSinfo.controllerStateSub);
%             velocities = statusMsg.Actual.Velocities;
%             if all(velocities<0.03)
%                 isMoving = 0;
%             else
%                 isMoving = 1;
%             end
        end
        
        % Display current job state
        function displayState(obj, message)
            disp(message);
            set(obj.Figure, 'NumberTitle', 'off', 'Name', message)
        end
        
        % Delete function
        function delete(obj)
            delete(obj.FlowChart)
        end
            
    end
  
end

