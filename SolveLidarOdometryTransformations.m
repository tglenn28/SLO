% ------------------------------------------------------------------------
%   Solve Trajectory of Point Cloud Odometry
%
% ------------------------------------------------------------------------
%   Course: EECS 568
%   Author: A. Jeffries
%   Date: 4/17/2020
clc;
clear all;
close all;
addpath('/usr/local/gtsam_toolbox/'); % Add GTSAM Toolbox to your path
import gtsam.*;

Tr =[2.347736981471e-04 -9.999441545438e-01 -1.056347781105e-02 -2.796816941295e-03;
     1.044940741659e-02  1.056535364138e-02 -9.998895741176e-01 -7.510879138296e-02;
     9.999453885620e-01  1.243653783865e-04  1.045130299567e-02 -2.721327964059e-01;
     0                   0                   0                   1];

%% Step 1: Option 1 - Read in the Files or Load in MAT File
Option = 1;
removedMoving = true;
if Option == 1
    fileLocationKITTI = '/Users/ajeffries/Documents/School/EECS_568/Project/data/07/07.txt';
    fileLocationSEMANTICKITTI = '/Users/ajeffries/Documents/School/EECS_568/Project/data/07/poses.txt';
    % Find all text files in the following directory
    files = dir('/Users/ajeffries/Documents/School/EECS_568/Project/data/07/07_moving_labels_removed/*.txt');
    
    T_Pose_GT = readPOSEfromKITTI(fileLocationKITTI); % Read in Ground Truth Pose
    T_Pose_SK = readPOSEfromKITTI(fileLocationSEMANTICKITTI); % Read in Semantic Poses
    fileNames = {files.name}';
    
    % Loop through Files to Create the Edges from Point Cloud Transformation Files
    edges_sicp = [];
    edges_gicp = [];
    for iFile=1:numel(files)
        tmpFile = files(iFile).name;
        % Find Source and Target
        expression = '(\d*)_(\d*).txt';
        tokens = regexp(tmpFile,expression,'tokens');
        if ~isempty(tokens)
            source = str2double(tokens{1}{1});
            target = str2double(tokens{1}{2});
            
            fileWpath = fullfile(files(iFile).folder,tmpFile);
            [T_SICP, T_GICP, COV] = loadTransformationsAndCov(fileWpath);
            edges_sicp = [edges_sicp; source target reshape(T_SICP.',1,[])];
            edges_gicp = [edges_gicp; source target reshape(T_GICP.',1,[])];
        end
    end
    
    if removedMoving
        % Find all text files in the following directory
        files = dir('/Users/ajeffries/Documents/School/EECS_568/Project/data/07/07_no_labels_removed/*.txt');
        
        % Loop through Files to Create the Edges from Point Cloud Transformation Files
        edges_sic_mov_rm = [];
        edges_gicp_mov_rm = [];
        for iFile=1:numel(files)
            tmpFile = files(iFile).name;
            % Find Source and Target
            expression = '(\d*)_(\d*).txt';
            tokens = regexp(tmpFile,expression,'tokens');
            if ~isempty(tokens)
                source = str2double(tokens{1}{1});
                target = str2double(tokens{1}{2});
                
                fileWpath = fullfile(files(iFile).folder,tmpFile);
                [T_SICP, T_GICP, COV] = loadTransformationsAndCov(fileWpath);
                edges_sic_mov_rm = [edges_sic_mov_rm; source target reshape(T_SICP.',1,[])];
                edges_gicp_mov_rm = [edges_gicp_mov_rm; source target reshape(T_GICP.',1,[])];
            end
        end
    end
    
    %save('transformations&GT.mat','edges_sic','edges_gicp','edges_gicp_mov_rm','edges_sic_mov_rm','T_Pose_GT','T_Pose_SK')
else
    %% Step 1: Option 2 - Alternatively Load in Required Matrices
    load('transformations&GT.mat');
end
%% Step 1.2: Creay Struct to Store 
% Step 2.1: Plot Ground Truth Pose Calc with Lidar Odometry
GT_cumm_dist = 0;
pt_GT = [T_Pose_GT(1,5) T_Pose_GT(1,9) T_Pose_GT(1,13)];
pt_GT_s = [T_Pose_SK(1,5) T_Pose_SK(1,9) T_Pose_SK(1,13)];
for i=2:size(T_Pose_GT,1)
    pose = [reshape(T_Pose_GT(i,2:end),4,3)'; 0 0 0 1];
    
    pt_GT = [pt_GT; [T_Pose_GT(i,5) T_Pose_GT(i,9) T_Pose_GT(i,13)]];
    
    pt_GT_s = [pt_GT_s; [T_Pose_SK(i,5) T_Pose_SK(i,9) T_Pose_SK(i,13)]];
    
    GT_cumm_dist = [GT_cumm_dist; sum(vecnorm(pt_GT(1:i-1,:)' - pt_GT(2:i,:)'))];
end
% Creay Struct to Store 
PoseStruct(1).name = 'KITTI Pose Ground Truth';
PoseStruct(1).poses = T_Pose_GT; % NOTE: Removing 0, 0, 0 pose since we dont calculate it
PoseStruct(2).name = 'Semantic KITTI Pose Ground Truth';
PoseStruct(2).poses = T_Pose_SK;

%% Step 2: Do Lidar Odometry & Solve with iSAM2
noiseMod = noiseModel.Diagonal.Sigmas([0.1; 0.1; 0.1; 0.1; 0.1; 0.1]);

%iSAM2
PoseStruct(3).name = 'SICP Lidar Poses';
PoseStruct(3).poses= loopThroughPosesISAM2(T_Pose_GT, edges_sicp,noiseMod,Tr);

PoseStruct(4).name = 'GICP Lidar Poses';
PoseStruct(4).poses = loopThroughPosesISAM2(T_Pose_GT, edges_gicp,noiseMod, Tr);
if removedMoving
    PoseStruct(5).name = 'SICP Lidar Poses with Removed Moving Classes';
    PoseStruct(5).poses = loopThroughPosesISAM2(T_Pose_GT, edges_sic_mov_rm,noiseMod,Tr);
    
    PoseStruct(6).name = 'GICP Lidar Poses with Removed Moving Classes';
    PoseStruct(6).poses = loopThroughPosesISAM2(T_Pose_GT, edges_gicp_mov_rm,noiseMod, Tr);
end

save('PoseStruct07.mat','PoseStruct');

%% Step 3: Plot Everything for Animation
animated = true;
addPointCloud = false;
PCDDir = dir('/Volumes/DRAGON/KITTI/KITTI_MOD/07/velodyne/*.pcd');
PCDFiles = fullfile({PCDDir.folder}', {PCDDir.name}');
%%
M = plotPoses4VideoandCalcError(PoseStruct, 1, animated,'07Animation',addPointCloud,PCDFiles);





function lidar_poses = loopThroughPosesISAM2(T_pose, edges,noiseMod,Tr)

% Import GTSAM Library
import gtsam.*;
%% Create iSAM2, graph container and add factors to it
isamParams = ISAM2Params;
isamParams.setFactorization('CHOLESKY');

isam = ISAM2(ISAM2Params);

graph = NonlinearFactorGraph; % Edges
initialEstimate = Values; % Poses
%% Intialize
poses_range = min(edges(:,2)) : max(edges(:,1)); % This assumed No Skipped Poses
P0 = [reshape(T_pose(1,2:end),4,3)'; 0 0 0 1];

% Add Prior to Graph
graph.add(PriorFactorPose3(T_pose(1,1), Pose3(P0),noiseMod)); % add directly to graph

% Add Initial Odometry Estimate
initialEstimate.insert(T_pose(1,1), Pose3(P0));
isam.update(graph, initialEstimate);
result = isam.calculateEstimate();

%% Loop Through Poses
iVertex = 0;
tmp_pose_mat = (result.atPose3(iVertex).matrix)'; % Convert into Row Vector
lidar_poses = [poses_range(iVertex + 1) tmp_pose_mat(:)'];



for iVertex = 1: numel(poses_range) - 1
    
    graph = NonlinearFactorGraph;
    initialEstimate = Values;
    
    prevPose = result.atPose3(iVertex - 1);
    initialEstimate.insert(iVertex, prevPose);
    
    % Conditon 1 checks the position of teh 1st Edge
    condition1 = edges(:,1) == poses_range(iVertex + 1);
    % Condition2 Removes Point Clouds Ahead in Time of Pose. You can only look back...
    condition2 = edges(:,2) <= poses_range(iVertex + 1);
    
    edgesConnected2Source = edges(condition1&condition2,:);
    
     
    
    for jEdge=1:size(edgesConnected2Source,1)
        
        
        T = reshape(edgesConnected2Source(jEdge,3:end),4,4)';
        T = Tr*T/Tr;
        % Do Consecutive Check
        consecutive  = (edgesConnected2Source(jEdge,1) - edgesConnected2Source(jEdge,2)) == 1;
        
        
        
        if consecutive 
            % Define Covariance Weghted for Consecutive Edges
            noiseModWeighted = noiseModel.Diagonal.Sigmas([0.1; 0.1; 0.1; 0.1; 0.1; 0.1].*jEdge^2);
            graph.add(BetweenFactorPose3(edgesConnected2Source(jEdge,2), ...
                edgesConnected2Source(jEdge,1), Pose3(T), noiseModWeighted));
        else
            %graph.add(BetweenFactorPose3(edgesConnected2Source(jEdge,1), ...
            %edgesConnected2Source(jEdge,2), Pose3(T), noiseMod));
        end
    end
    
    isam.update(graph, initialEstimate);
    result = isam.calculateEstimate();
    tmp_pose_T = (result.atPose3(iVertex).matrix)'; % Convert into Row Vector
    
    
    lidar_poses = [lidar_poses; poses_range(iVertex + 1) tmp_pose_T(:)'];
    
    
end

end
