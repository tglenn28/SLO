clear all;
close all;

%% Step 1: Option 1 - Read in the Files or Load in MAT File
fileLocation = '~/Desktop/Project/poses/00.txt';
% Find all text files in the following directory
files = dir('~/Desktop/Project/00_new_2/00_first_3000/*.txt');

T_Pose_GT = readPOSEfromKITTI(fileLocation); % Read in Ground Truth Pose
fileNames = {files.name}';

% Loop through Files to Create the Edges from Point Cloud Transformation Files
edges_sic = [];
edges_gicp = [];
cov = [];
for iFile=1:numel(files)
    tmpFile = files(iFile).name;
    % Find Source and Target
    expression = '(\d*)_(\d*).txt';
    tokens = regexp(tmpFile,expression,'tokens');
    source = str2double(tokens{1}{1});
    target = str2double(tokens{1}{2});
        
    fileWpath = fullfile(files(iFile).folder,tmpFile);
    [T_SICP, T_GICP, COV] = loadTransformationsAndCov(fileWpath);
    
    edges_sic = [edges_sic; source target reshape(T_SICP.',1,[])];
    edges_gicp = [edges_gicp; source target reshape(T_GICP.',1,[])];
%     cov = [cov; source target reshape(COV.',1,[])];
end
save('transformations&GT_00_new.mat','edges_sic','edges_gicp','T_Pose_GT')