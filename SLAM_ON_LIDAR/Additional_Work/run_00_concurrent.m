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

GroundTruthPath = "~/Desktop/Project/poses/00.txt";

import gtsam.*;
import gtsam_unstable.*;
%truth trajectory of 07 we find 1 loops closure per belot.size()-1

%% load data 
datapath = '~/Desktop/Project/Final/Tranformation_Data/transformations&GT_00_new.mat';
% make sure transformations&GT.mat is in the current directory%    0.5331

[edges_gicp_sort, edges_gicp_con, edges_gicp_non, edgelidar_poses_sicps_sicp_sort, edges_sicp_con, edges_sicp_non, T_Pose_GT, edges_sicp, edges_gicp] = AliloadTransformations(datapath);

%% Calculate ground truth
gt3 = T_Pose_GT;
gt = [];
for i=1:size(gt3,1)
    pose = [reshape(gt3(i,2:end),4,3)'; 0 0 0 1];
    gt = [gt; pose(1:3,4)'];
end

%% Define transformation
Tr =   [2.347736981471e-04 -9.999441545438e-01 -1.056347781105e-02 -2.796816941295e-03;
    1.044940741659e-02 1.056535364138e-02 -9.998895741176e-01 -7.510879138296e-02;
    9.999453885620e-01 1.243653783865e-04 1.045130299567e-02 -2.721327964059e-01;
    0                   0                   0                   1];

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

%% Run Concurrent filter; plot and save results
Noise =[0.1; 0.1; 0.1; 0.1; 0.1; 0.1];
priorNoise = [0.1; 0.1; 0.1; 0.1; 0.1; 0.1];
lag = 10;
sync = 20;
icp = "gicp";
non_con = 3;
printresult = 1;
color = 'm';

edges_non = edges_sicp_non;

edges_con = edges_sicp_con;


AliConcurrent503aided2_latest(lag, sync, priorNoise, Noise, edges_non, edges_con, T_Pose_GT, non_con, Tr, printresult, loopclosures, color)
