
GroundTruthPath = "~/Desktop/Project/poses/03.txt";

import gtsam.*;
import gtsam_unstable.*;

%% load data 
% make sure transformations&GT.mat is in the current directory
[edges_gicp_con, edges_gicp_non, cov_gicp_con, cov_gicp_non, edges_sicp_con, edges_sicp_non, cov_sicp_con, cov_sicp_non, T_Pose_GT, edges_sicp, edges_gicp] = AliloadTransformations03('~/Desktop/Project/Final/Tranformation_Data/transformations&GT03.mat');
% [edges_gicp_sort, edges_gicp_con, edges_gicp_non, edges_sicp_sort, edges_sicp_con, edges_sicp_non, T_Pose_GT, edges_sicp, edges_gicp] = AliloadTransformations('~/Desktop/Project/Final/Tranformation_Data/transformations&GT.mat');


%% Define loop closures

%% Run Concurrent filter; plot and save results
Noise =[0.1; 0.1; 0.1; 0.1; 0.1; 0.1];
priorNoise = [0.1; 0.1; 0.1; 0.1; 0.1; 0.1];
lag = 2;
sync = 5;
icp = "gicp";
non_con = 4;
printresult = 1;
loopclosures = 0;

%for gicp
[Results, smootherResult] = AliConcurrent503aided2(lag, sync, priorNoise, Noise, icp, non_con, printresult, loopclosures);

results_gicp = [];
for i=1:size(smootherResult,2)    
    pose = matrix(Pose3(smootherResult(i)));
    results_gicp = [results; pose(1:3,4)'];   
end
results_gicp = vertcat([0,0,0],results_gicp);

%for sicp
icp = "icp";
[Results2, smootherResult2] = AliConcurrent503aided2(lag, sync, priorNoise, Noise, icp, non_con, printresult, loopclosures);

results_sicp = [];
for i=1:size(smootherResult2,2)    
    pose = matrix(Pose3(smootherResult2(i)));
    results_sicp = [results; pose(1:3,4)'];   
end
results_sicp = vertcat([0,0,0],results_sicp);


%% Plot ground truth
gt3 = readPOSEfromKITTI(GroundTruthPath);
gt = [];
for i=1:size(gt3,1)
    pose = [reshape(gt3(i,2:end),4,3)'; 0 0 0 1];
    gt = [gt; pose(1:3,4)'];
    
end

plot3(gt(:,1),gt(:,2),gt(:,3),'k');

%% ISAM portion

noiseMod = noiseModel.Diagonal.Sigmas([0.1; 0.1; 0.1; 0.1; 0.1; 0.1]);
lidar_poses_sic = loopThroughPosesiSAM(T_Pose_GT, edges_sicp,noiseMod);
lidar_poses_gicp = loopThroughPosesiSAM(T_Pose_GT, edges_gicp,noiseMod);

% Plot SIC
plot3(lidar_poses_sic(:,1),lidar_poses_sic(:,2),lidar_poses_sic(:,3),'g');
% Plot GICP
plot3(lidar_poses_gicp(:,1),lidar_poses_gicp(:,2),lidar_poses_gicp(:,3),'r');
% 
% grid on
% legend('show');

%% Find Euclidean Error
ErrorMean_gicp = AliMeanTranslationError(results_gicp,gt);
ErrorMean_sicp = AliMeanTranslationError(results_sicp,gt);
ErrorBoth = vertcat(ErrorMean_gicp,ErrorMean_sicp);


tempText = "ErrorResults" + "gicp" + "sicp" + lag + "_" + sync + "_" + non_con + ".mat";
save(tempText, 'ErrorBoth');

tempText = "FinalResults" + lag + "_" + sync + "_" + non_con + icp + ".fig";
saveas(gcf,tempText);

















