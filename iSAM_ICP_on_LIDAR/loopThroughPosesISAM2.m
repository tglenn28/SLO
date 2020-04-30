% ------------------------------------------------------------------------
%   iSAM2 Implemenatation of Point Cloud Odometry
%
% ------------------------------------------------------------------------
%   Course: EECS 568
%   Author: A. Jeffries & Ali Badreddine
%   Date: 3/19/2020

function lidar_poses = loopThroughPosesISAM2(ax,colors,name,T_pose, edges,noiseMod,Tr,loopclosures)

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
priornoiseMod = noiseModel.Diagonal.Sigmas([0; 0; 0; 0; 0; 0]);
graph.add(PriorFactorPose3(T_pose(1,1), Pose3(P0),priornoiseMod)); % add directly to graph

% Add Initial Odometry Estimate
initialEstimate.insert(T_pose(1,1), Pose3(P0));
isam.update(graph, initialEstimate);
result = isam.calculateEstimate();

%% Loop Through Poses
iPose = 0;
tmp_pose_mat = (result.atPose3(iPose).matrix)'; % Convert into Row Vector
lidar_poses = tmp_pose_mat(:)';


whole_plot = plot3(ax,lidar_poses(:,4),lidar_poses(:,8),lidar_poses(:,12),colors,'DisplayName',name);
legend(ax,'show');

edges = vertcat(edges,loopclosures);

for iPose = 1: max(edges(:,1))%size(T_pose,1)-1
    
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
        
        T = Tr*T*inv(Tr);
        graph.add(BetweenFactorPose3(edgesConnected2Source(jEdge,2), ...
            edgesConnected2Source(jEdge,1), Pose3(T), noiseMod));
    end
    
    isam.update(graph, initialEstimate);
    result = isam.calculateEstimate();
    tmp_pose_T = (result.atPose3(iPose).matrix)'; % Convert into Row Vector
    
    lidar_poses = [lidar_poses; tmp_pose_T(:)'];
    
    set(whole_plot,'XData',lidar_poses(:,4),'YData',...
        lidar_poses(:,8),'ZData',lidar_poses(:,12));
    
    
    %axis equal
    %axis tight
    grid on
    view(-180,0)
    %pause(0.001)
end

end














% 
%  Tr =   [2.347736981471e-04 -9.999441545438e-01 -1.056347781105e-02 -2.796816941295e-03;
%     1.044940741659e-02 1.056535364138e-02 -9.998895741176e-01 -7.510879138296e-02;
%     9.999453885620e-01 1.243653783865e-04 1.045130299567e-02 -2.721327964059e-01;
%     0                   0                   0                   1];
% 
% 
% results = zeros(size(result),16);
% for i=1:size(result)-1
%     m = result.atPose3(i).matrix;
%     m = (Tr * m) / Tr;
%     results(i,:) = m(:)';
%     
% end
% 
% lidar_poses = zeros(size(results,1),3);
% 
% for i = 1:size(results)
%         lidar_poses(i,1) = results(i,4);
%     lidar_poses(i,2) =results(i,8);
%     lidar_poses(i,3) = results(i,12);
% 
% end
%     


