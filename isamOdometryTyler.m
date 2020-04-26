clc

import gtsam.*
import gtsam_unstable.*

dataset = "03";
directory = "~/rob530proj/data/transformations/";

numPoses = 800; %999;
numConnections = 1;

%%%%%%%%%%%%%%%%%%%%%%%%% KITTI Poses %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
k_poses = import_poses(strcat(strcat("/home/tyler/rob530proj/data/poses/", dataset), ".txt"), [1, Inf]);
k_poseMats = zeros(4,4,numPoses + 1);
for i = 1:size(k_poseMats,3) %numPoses
    r = k_poses(i,:);
    k_poseMats(:,:,i) = [r.p11 r.p12 r.p13 r.p14;
                       r.p21 r.p22 r.p23 r.p24;
                       r.p31 r.p32 r.p33 r.p34;
                       0     0     0     1];
end

k_positions = zeros(3,size(k_poseMats,3));
k_positions(1,:) = k_poseMats(1,4,:);
k_positions(2,:) = k_poseMats(2,4,:);
k_positions(3,:) = k_poseMats(3,4,:);
k_positions = k_positions(:,2:end); %removing the first zero pose

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%% SemanticKITTI Poses %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
sk_poses = import_poses(strcat(strcat("/home/tyler/rob530proj/data/labels/", dataset), "/poses.txt"), [1, Inf]);
sk_poseMats = zeros(4,4,numPoses);
for i = 1:size(sk_poseMats,3) %numPoses
    r = sk_poses(i,:);
    sk_poseMats(:,:,i) = [r.p11 r.p12 r.p13 r.p14;
                       r.p21 r.p22 r.p23 r.p24;
                       r.p31 r.p32 r.p33 r.p34;
                       0     0     0     1];
end

sk_positions = zeros(3,size(sk_poseMats,3));
sk_positions(1,:) = sk_poseMats(1,4,:);
sk_positions(2,:) = sk_poseMats(2,4,:);
sk_positions(3,:) = sk_poseMats(3,4,:);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

tic

smootherLag = 1.0;
params = gtsam.ISAM2Params;
isam = ISAM2(params);
newValues = Values;

% Create graph container and add factors to it
newFactors = NonlinearFactorGraph;

% Add prior
priorNoise = noiseModel.Diagonal.Sigmas([0; 0; 0; 0; 0; 0]);
prior = Pose3(eye(4));
newFactors.add(PriorFactorPose3(0,prior, priorNoise)); % add directly to graph
newValues.insert(0, prior);

Ts = zeros(4,4,numPoses);
Ts_GICP = zeros(4,4,numPoses);
posesIntegrated = zeros(4,numPoses);
posesInteg = zeros(4,4,numPoses);
posesIntegGICP = zeros(4,4,numPoses);
pathLength = zeros(numPoses,1);
pathLengthGT = zeros(numPoses,1);

Tr = [0.0002 -0.9999 -0.0106 -0.0028;
      0.0104  0.0106 -0.9999 -0.0751;
      0.9999  0.0001  0.0105 -0.2721;
           0       0       0       1];
       
Tr =   [2.347736981471e-04 -9.999441545438e-01 -1.056347781105e-02 -2.796816941295e-03;
        1.044940741659e-02 1.056535364138e-02 -9.998895741176e-01 -7.510879138296e-02;
        9.999453885620e-01 1.243653783865e-04 1.045130299567e-02 -2.721327964059e-01;
        0                   0                   0                   1];

for i=1:numPoses    
    %%%%%%%%%%% source %%%%%%%%%%%%%%
    numZeros = 6 - numel(num2str(i));

    fileStr = "";
    for k = 1:numZeros
       fileStr = strcat(fileStr, "0");
    end
    sourceFileStr = strcat(fileStr, num2str(i));
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
    for j = 1:numConnections
        if (i - j >= 0)
            %%%%%%%%%%% target %%%%%%%%%%%%%%
            numZeros = 6 - numel(num2str(i-j));

            fileStr = "";
            for k = 1:numZeros
               fileStr = strcat(fileStr, "0");
            end
            targetFileStr = strcat(fileStr, num2str(i-j));
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            filePath = strcat(strcat(strcat(strcat(strcat(strcat(directory, dataset),"/"), sourceFileStr),"_"),targetFileStr),".txt");
%             [T_SICP, T_GICP] = loadTransformations(filePath);
%             [T_SICP, T_GICP, MSE] = loadTransformationsMSE(filePath);
            [T_SICP, T_GICP, cov] = loadTransformationsCov(filePath);
            T_SICP = Tr*T_SICP*inv(Tr);
            T_GICP = Tr*T_GICP*inv(Tr);
            noiseCov = 0.1;
            
            noise = noiseModel.Gaussian.Covariance(j^2*noiseCov*diag([1, 1, 1, 1, 1, 1]));

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%             chol(cov);
%             noise = noiseModel.Gaussian.Covariance(cov);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            newFactors.add(BetweenFactorPose3(i-j, i, Pose3(T_SICP), noise));
            
            if j == 1
                Ts(:,:,i) = T_SICP;
                Ts_GICP(:,:,i) = T_GICP;
            end
        end
    end
    
    if i > 1
        posesInteg(:,:,i) = posesInteg(:,:,i-1)*Ts(:,:,i);
        posesIntegGICP(:,:,i) = posesIntegGICP(:,:,i-1)*Ts_GICP(:,:,i);
        newValues.insert(i, result.at(i-1));
    else
        newValues.insert(i, compose(Pose3(T_SICP), prior));
        posesInteg(:,:,i) = eye(4)*Ts(:,:,i);
        posesIntegGICP(:,:,i) = eye(4)*Ts_GICP(:,:,i);
    end
    isam.update(newFactors, newValues);
    result = isam.calculateEstimate();
    
    newValues.clear();
    newFactors = NonlinearFactorGraph;
end

toc

positions = zeros(3, numPoses);
positionsUnopt = zeros(3, numPoses);
positionsGicp = zeros(3, numPoses);
for i = 1:numPoses
   if i == 1
       pathLength(i) = norm(Ts(1:3,4,i));
       trans = inv(k_poseMats(:,:,i))*k_poseMats(:,:,i+1);
       pathLengthGT(i) = norm(trans(1:3,4));
   else
       pathLength(i) = pathLength(i-1) + norm(Ts(1:3,4,i));
       trans = inv(k_poseMats(:,:,i))*k_poseMats(:,:,i+1);
       pathLengthGT(i) = pathLength(i-1) + norm(trans(1:3,4));
   end
   point = result.at(i).translation();
   positions(:,i) = [point.x(), point.y(), point.z()]'; 
   positionsUnopt(:,i) = posesInteg(1:3,4,i);
   positionsGicp(:,i) = posesIntegGICP(1:3,4,i);
end

yAdjust = (10/2000)*[1:numPoses];
       
%% Position Errors
errSicpOpt = vecnorm(positions(1:3,:) - k_positions);
errSicpUnopt = vecnorm(positionsUnopt - k_positions);
errGicp = vecnorm(positionsGicp(1:3,:) - k_positions);

percErrSicpOpt = errSicpOpt./pathLengthGT';
percErrSicpUnopt = errSicpUnopt./pathLengthGT';
percErrGicp = errGicp./pathLengthGT';

meanPercErrSicpOpt = mean(percErrSicpOpt)
meanPercErrSicpUnopt = mean(percErrSicpUnopt)
meanPercErrGicp = mean(percErrGicp)
%%
figure(1)
clf
plot3(positions(1,:), positions(2,:), positions(3,:));
axis equal
view(2)
hold on
plot3(sk_positions(1,:), sk_positions(2,:), sk_positions(3,:),'Color','g')
plot3(k_positions(1,:), k_positions(2,:), k_positions(3,:))%,'Color','m')
hold on
% plot3(posesIntegrated(1,:), posesIntegrated(2,:), posesIntegrated(3,:),'Color','r')
plot3(squeeze(posesInteg(1,4,:)), squeeze(posesInteg(2,4,:)), squeeze(posesInteg(3,4,:)))%,'Color','b')
plot3(squeeze(posesIntegGICP(1,4,:)), squeeze(posesIntegGICP(2,4,:)), squeeze(posesIntegGICP(3,4,:)),'Color','c')
title("Odometry Trajectory Comparisons")
legend('Optimized SICP', 'SemanticKITTI Ground Truth', 'KITTI Ground Truth', 'Unoptimized (sequential) SICP', 'Unoptimized (sequential) GICP')
view(3)
xlabel('x-position (m)')
ylabel('y-position (m)')
zlabel('z-position (m)')

figure(2)
clf
plot(errSicpOpt)
hold on
plot(errSicpUnopt)
legend('errSicpOpt', 'errSicpUnopt')

figure(3)
clf
plot(abs(positions(1,:) - k_positions(1,:)))
hold on
plot(abs(positions(2,:) - k_positions(2,:)))
plot(abs(positions(3,:) - k_positions(3,:)))
legend('x', 'y', 'z')
title('SICP Errors for each coordinate over trajectory')

figure(4)
clf
plot(abs(positionsGicp(1,:) - k_positions(1,:)))
hold on
plot(abs(positionsGicp(2,:) - k_positions(2,:)))
plot(abs(positionsGicp(3,:) - k_positions(3,:)))
legend('x', 'y', 'z')
title('GICP Errors for each coordinate over trajectory')
