%_________________________________________________________________________
%   iSAM on SICP & GICP transformation for Kitti dataset 07
%
%_________________________________________________________________________
%   Course: EECS 568
%   Author: Ali Badreddine
%   Date: 04/30/2020
%_________________________________________________________________________

clear all;
close all;

f = figure(1);
ax = axes('Parent',f);
hold(ax,'on');

%GroundTruthPath = "~/Desktop/Project/poses/00.txt";

import gtsam.*;
import gtsam_unstable.*;
%truth trajectory of 07 we find 1 loops closure per belot.size()-1

%% load data 
datapath = '~/Desktop/Project/Final/Tranformation_Data/transformations&GT_00_new.mat';
% make sure transformations&GT.mat is in the current directory%    0.5331

[edges_gicp_sort, edges_gicp_con, edges_gicp_non, edgelidar_poses_sicps_sicp_sort, edges_sicp_con, edges_sicp_non, T_Pose_GT, edges_sicp, edges_gicp] = AliloadTransformations(datapath);

%% Calculate ground truth
% gt3 = readPOSEfromKITTI(GroundTruthPath);
%OR
gt3 = T_Pose_GT;

gt = [];
for i=1:size(gt3,1)
    pose = [reshape(gt3(i,2:end),4,3)'; 0 0 0 1];
    gt = [gt; pose(1:3,4)'];
end

%% Plot ground truth
hold on
plot3(gt(:,1),gt(:,2),gt(:,3),'k');

%% Define loop closures
%Evaluating the euclidean norm among non consecutive poits in the groundb

% closureidx = findloopclosure(gt3, 0.3, 0); %to generate loop closures
%OR
load("loopclosures_less_0.5_00.mat");

%find loop closures, whcih indexes with euclidean error <0.2 are 100 poses
%or greater apart
diff = closureidx(:,1) - closureidx(:,2);
test = horzcat(closureidx,diff);
sort = sortrows(test,3);
closureidx2 = sort(sort(:,3)>=400,:);
closureidx3 = closureidx2(:,1:2);

loopclosures = [];


%% Find transformations for loop closures
for i = 1:size(closureidx3,1)
    
closure1 = closureidx3(i,1);
closure2 = closureidx3(i,2);

T =  [reshape(gt3(gt3(:,1)==closure1,2:end),4,3)'; 0 0 0 1] \ ([reshape(gt3(gt3(:,1)==closure2,2:end),4,3)'; 0 0 0 1]);

loopclosures = [loopclosures; closure1, closure2, T(:)'];

end

loopclosures = loopclosures(loopclosures(:,1)<=3000,:);
loopclosures = loopclosures(155<=loopclosures(:,1),:);

max_traj = max(loopclosures(:,1));
max_traj = 3010;


loopclosures = loopclosures(loopclosures(:,1)<=max_traj,:);

% loopclosures = [];


%% Define transformation
Tr =   [2.347736981471e-04 -9.999441545438e-01 -1.056347781105e-02 -2.796816941295e-03;
    1.044940741659e-02 1.056535364138e-02 -9.998895741176e-01 -7.510879138296e-02;
    9.999453885620e-01 1.243653783865e-04 1.045130299567e-02 -2.721327964059e-01;
    0                   0                   0                   1];

%% ISAM portion


% edges_sicp_con = vertcat(edges_sicp_con, edges_sicp_con(edges_sicp_con(:,size(edges_sicp_con,2))==3,:));
noiseMod = noiseModel.Diagonal.Sigmas([0.1; 0.1; 0.1; 0.1; 0.1; 0.1]);

edges_sicp_con = edges_sicp_con(155<=edges_sicp_con(:,1),:);
edges_sicp_con = edges_sicp_con(edges_sicp_con(:,1)<=max_traj,:);
edges_sicp_con(:,1) = edges_sicp_con(:,1)-154;
edges_sicp_con(:,2) = edges_sicp_con(:,2)-154;

edges_gicp_con = edges_gicp_con(155<=edges_gicp_con(:,1),:);
edges_gicp_con = edges_gicp_con(edges_gicp_con(:,1)<=max_traj,:);
edges_gicp_con(:,1) = edges_gicp_con(:,1)-154;
edges_gicp_con(:,2) = edges_gicp_con(:,2)-154;

T_Pose_GT_new = T_Pose_GT(154<=T_Pose_GT(:,1),:);
T_Pose_GT_new = T_Pose_GT_new(T_Pose_GT_new(:,1)<=max_traj,:);
T_Pose_GT_new(:,1) = T_Pose_GT_new(:,1) - 154;

loopclosures(:,1) = loopclosures(:,1) - 154;
loopclosures(:,2) = loopclosures(:,2) - 154;

lidar_poses_sicp = loopThroughPosesISAM2(ax,'g','SICP - iSAM2',T_Pose_GT_new, edges_sicp_con(:,1:end-1),noiseMod, Tr, loopclosures);
lidar_poses_gicp = loopThroughPosesISAM2(ax,'r','GICP - iSAM2',T_Pose_GT_new, edges_gicp_con(:,1:end-1),noiseMod, Tr, loopclosures);

axis equal;
view(-180,0)
axis tight

tempText = "iSAM_sicp" + ".mat";
save(tempText, 'lidar_poses_sicp');

tempText = "iSAM_gicp" + ".mat";
save(tempText, 'lidar_poses_gicp');

%% Find Euclidean Error
% for 07 data we only look at theFinalResultsPlot first 3010 poses

% g = figure;
% ag = axes('Parent',g);
gt3 = T_Pose_GT_new;

gt = [];
for i=1:size(gt3,1)
    pose = [reshape(gt3(i,2:end),4,3)'; 0 0 0 1];
    gt = [gt; pose(1:3,4)'];
end

ErrorMean_gicp_isam = AliMeanTranslationError('r','GICP - iSAM2',horzcat(lidar_poses_gicp(:,4),lidar_poses_gicp(:,8),lidar_poses_gicp(:,12)),gt);
ErrorMean_sicp_isam = AliMeanTranslationError('g','SICP - iSAM2',horzcat(lidar_poses_sicp(:,4),lidar_poses_sicp(:,8),lidar_poses_sicp(:,12)),gt);
ErrorBoth_isam = vertcat(ErrorMean_gicp_isam,ErrorMean_sicp_isam);


%save mean translation error gicp first number, sicp second number
tempText = "Error_iSAM" + ".mat";
save(tempText, 'ErrorBoth_isam');

%% Save Plots
%save trajectory
figure(1)
tempText = "Trajectory"+".fig";
saveas(gcf,tempText);

%save Norm Translation Error
figure(2)
tempText = "TranslationError"+".fig";
saveas(gcf,tempText);

%save l2 Norm Error
figure(3)
tempText = "NormDiff"+".fig";
saveas(gcf,tempText);

%save Distance Metric
figure(4)
tempText = "DistanceMetric"+".fig";
saveas(gcf,tempText);





