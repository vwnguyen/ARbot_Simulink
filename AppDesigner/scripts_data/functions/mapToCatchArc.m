%% TARGET FINALIZATION
% map target from camera frame to base of robot, 

% Params:
% P_C = array of targets in the camera reference frame [ X Y Z time_spotted ] 
% P_B_CORG = vector from robot base -> catching line [ X Y Z ]
% belt_rate = speed of belt in m/s
% max_Catching_time = maximum time it takes to catch and sort the waste
% catching arc = 3D points where the workspace is
% ikSols = lookup table for ik solutions

% Outputs: 
% catching coordinates = an array with time of arrival

function [P_B  distanceToCatchLine timeToCatchLine ikSol P_C P_W] = mapToCatchArc(P_B_CORG,P_C,belt_rate,rotation_matrix,max_Catching_Time,eeOrientation,dist_To_Catch,catching_arc,ikSols)

    % get the number of targets 
    target_Size = size(P_C);
    T_B_C = [ 0 0 0 0;
      0 0 0 0;
      0 0 0 0;
      0 0 0 1;
      ];

    % where at the catching line it should end effector go 
    T_B_C(1:3,1:3) = rotation_matrix; % rotation matrix to camera
    T_B_C(1:3,4) = P_B_CORG; % vector to camera origin
    P_W = []; % array of workspace points from robot POV to camera frame
    catching_arc(:,4) = 1;
    % describe points from the base in the camera ref frame
    for i=1:length(catching_arc)
        P_W(i,:) = inv(T_B_C) * catching_arc(i,:).';
        % P_W(i,2) = P_W(i,2) + dist_To_Catch;
    end
    
    P_Final = [];
    
    for i=1:target_Size(1)
        % map the targets from base into the camera frame
        % min value is the closest Y
        [minValue,closestIndex,closestValue] = findClosest(P_C(i,1),P_W);
        closeIndexFinal = closestIndex(1);
        
        % estimate distance and time to catching line
        distanceToCatchLine(i) = P_W(closeIndexFinal,2) - P_C(i,2);
        timeToCatchLine(i) = distanceToCatchLine(i) / belt_rate; 
        
        % physically move the target to the catching arc
        P_Final(i,1) = P_W(closeIndexFinal,1);
        P_Final(i,2) = P_W(closeIndexFinal,2);
        P_Final(i,3) = P_W(closeIndexFinal,3);
        P_Final(i,4) = 1;
        P_B(i,:) = T_B_C * P_Final(i,:).'; % vector from the base of the robot to target

        ikSol(i,:) = inverseKineRBT(P_B(i,1),P_B(i,2),P_B(i,3),eeOrientation);
    end

end