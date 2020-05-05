%% TARGET FINALIZATION
% map target from camera frame to base of robot, 

% Params:
% P_C = array of targets in the camera reference frame [ X Y Z time_spotted ] 
% P_B_CORG = vector from robot base -> catching line [ X Y Z ]
% belt_rate = speed of belt in m/s
% max_Catching_time = maximum time it takes to catch and sort the waste

% Outputs: 
% catching coordinates = an array with time of arrival

function [P_B  distanceToCatchLine timeToCatchLine ikSol P_C] = mapToCatchLineSim(P_B_CORG,P_C,belt_rate,rotation_matrix,max_Catching_Time,eeOrientation,dist_To_Catch)

    target_Size = size(P_C);
    T_B_C = [ 0 0 0 0;
      0 0 0 0;
      0 0 0 0;
      0 0 0 1;
      ];

    % where at the catching line it should end effector go 
    T_B_C(1:3,1:3) = rotation_matrix;
    T_B_C(1:3,4) = P_B_CORG;
    
    for i=1:target_Size(1)
        % estimate distance and time to catching line
        distanceToCatchLine(i) = dist_To_Catch - P_C(i,2);
        timeToCatchLine(i) = distanceToCatchLine(i) / belt_rate; 
        
        % physically move the target to the catching line
        P_C(i,2) = P_C(i,2) + distanceToCatchLine(i);
        
        P_B(i,:) = T_B_C * P_C(i,:).'; % vector from the base of the robot to target
        % ikSol = inverseKineRBT(P_B(i,1),P_B(i,2) ,P_B(i,3),eeOrientation);
        % P_B(i,:) = ikSol;
        ikSol(i,:) = inverseKineRBT(P_B(i,1),P_B(i,2),P_B(i,3),eeOrientation)
    end

    
    % ikSol = inverseKineRBT(P_A(1) , P_A(2) ,P_A(3),angleInRadians);
end