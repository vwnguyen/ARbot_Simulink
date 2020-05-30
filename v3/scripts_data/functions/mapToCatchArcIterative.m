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

function [P_B  distanceToCatchLine timeToCatchLine ikSol P_C P_W] = mapToCatchArcIterative(P_B_CORG,P_C,belt_rate,rotation_matrix,max_Catching_Time,dist_To_Catch,catching_arc,ikSols,time_coeff,dist_coeff)

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
    end
    
    P_Final = [];
    timeToCatchLine = [];
    for i=1:target_Size(1)
        % map the targets from base into the camera frame
        % min value is the closest Y
        [minValue,closestIndex,closestValue] = findClosest(P_C(i,1),P_W);
        closeIndexFinal = closestIndex(1);
        
        % estimate distance and time to catching line
%         distanceToCatchLine(i) = P_W(closeIndexFinal,2) - P_C(i,2);
%         timeToCatchLine(i) = distanceToCatchLine(i) / belt_rate; 
        target_xf = P_W(closeIndexFinal,2);
        target_xo = P_C(i,2);
        [ distanceToCatchLine(i) timeToCatchLine_unfiltered(i)]= ...
             calculateToA(target_xo,target_xf,time_coeff,dist_coeff,belt_rate);
        
        
        % physically move the target to the catching arc
        P_Final(i,1) = P_W(closeIndexFinal,1);
        P_Final(i,2) = P_W(closeIndexFinal,2);
        P_Final(i,3) = P_W(closeIndexFinal,3);
        P_Final(i,4) = 1;
        P_B(i,:) = T_B_C * P_Final(i,:).'; % vector from the base of the robot to target

        ikSol(i,:) = ikSols(closeIndexFinal,:);
    end
    
    % append initial target to the time array
%     timeToCatchLine = [timeToCatchLine timeToCatchLine_unfiltered(1)];
%     prevIdx = 1;
%     for i = 2:length(timeToCatchLine_unfiltered)
%         deltaT = timeToCatchLine_unfiltered(i) - timeToCatchLine(prevIdx);
%         if deltaT >= max_Catching_Time
%             timeToCatchLine = [timeToCatchLine timeToCatchLine_unfiltered(i)];
%             prevIdx = prevIdx + 1;
%         end
%     end
       
    % scan targets times until there is a viable initial target
    for i = 1:length(timeToCatchLine_unfiltered)
        deltaT = timeToCatchLine_unfiltered(i);
        if deltaT >= max_Catching_Time
            timeToCatchLine = [timeToCatchLine timeToCatchLine_unfiltered(i)];
            break
        end
    end

    for j = i+1:length(timeToCatchLine_unfiltered)
        deltaT = timeToCatchLine_unfiltered(j) - timeToCatchLine_unfiltered(j-1);
        if deltaT >= max_Catching_Time
            timeToCatchLine = [timeToCatchLine timeToCatchLine_unfiltered(j)];
        end
    end
    
end