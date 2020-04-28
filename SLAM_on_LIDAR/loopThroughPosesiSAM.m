% ------------------------------------------------------------------------
%   iSAM2 Implemenatation of Point Cloud Odometry
%
% ------------------------------------------------------------------------
%   Course: EECS 568
%   Author: A. Jeffries
%   Date: 3/19/2020

function lidar_poses = loopThroughPosesiSAM(T_pose, edges,noiseMod)

% Import GTSAM Library
import gtsam.*;
%% Create iSAM2, graph container and add factors to it
isamParams = ISAM2Params;
isamParams.setFactorization('CHOLESKY');

isam = ISAM2(ISAM2Params);

graph = NonlinearFactorGraph; % Edges
initialEstimate = Values; % Poses
%% Intialize
P0 = [reshape(T_pose(1,2:end),4,3)'; 0 0 0 1];

% Add Prior to Graph
graph.add(PriorFactorPose3(T_pose(1,1), Pose3(P0),noiseMod)); % add directly to graph

% Add Initial Odometry Estimate
initialEstimate.insert(T_pose(1,1), Pose3(P0));
isam.update(graph, initialEstimate);
result = isam.calculateEstimate();

%% Loop Through Poses


for iPose = 1: max(edges(:,1)) - 1%size(T_pose,1)-1
    
    graph = NonlinearFactorGraph;
    initialEstimate = Values;
    
    prevPose = result.atPose3(T_pose(iPose,1));
    initialEstimate.insert(T_pose(iPose+1,1), prevPose);
    
    condition1 = edges(:,1) == T_pose(iPose+1,1);
    % Condition2 Removes Point Clouds Ahead in Time of Pose. You can only look back...
    condition2 = edges(:,2) <= T_pose(iPose+1,1);
    edgesConnected2Source = edges(condition1&condition2,:);
    
    for jEdge=1:size(edgesConnected2Source,1)
        
        
        
        T = reshape(edgesConnected2Source(jEdge,3:end),4,4)';
        graph.add(BetweenFactorPose3(edgesConnected2Source(jEdge,1), ...
            edgesConnected2Source(jEdge,2), Pose3(T), noiseMod));
    end
    
    isam.update(graph, initialEstimate);
    result = isam.calculateEstimate();
end
lidar_poses = zeros(size(T_pose,1),2);
for i=1:result.size()-1
    lidar_poses(i,1) = result.atPose3(i).x;
    lidar_poses(i,2) = result.atPose3(i).y;
    lidar_poses(i,3) = result.atPose3(i).z;
end
end




