function [T_SICP, T_GICP, COV] = loadTransformationsAndCov(fileLocation)
% ------------------------------------------------------------------------
%   loadTransformationsAndCov Function
%   Take in a file location for a .txt file with covariance (eitehr 7x7 or 6x6)
%   SICP, and GICP  transformations and return the the three arrays
%   
%   
% ------------------------------------------------------------------------
%   Course: EECS 568
%   Author: A. Jeffries
%   Date: 4/27/2020
% ------------------------------------------------------------------------

% Open the  File
fileID = fopen(fileLocation,'r'); % Open a File for Reading

% Check if 'Covariance' is written at the top
nums = textscan(fileID,'%f');
nums = nums{1};
if isempty(nums)
    this_line =fgets(fileID); % Gets Rid of C'Covariance'
    nums = textscan(fileID,'%f');
    nums = nums{1};
    if numel(nums) == 81 % % Checks if there is a 7x7 Cov
        COV  = reshape(nums(1:49),7,7);
        T_SICP = reshape(nums(50:65),4,4)';
        T_GICP = reshape(nums(66:81),4,4)';
    elseif numel(nums) == 68 % % Checks if there is a 6x6 Cov
        COV  = reshape(nums(1:36),6,6);
        T_SICP = reshape(nums(37:52),4,4)';
        T_GICP = reshape(nums(53:68),4,4)';
    end
else % Otherwise we should just have GICP and SICP
    if numel(nums) == 32
    T_SICP = reshape(nums(1:16),4,4)';
    T_GICP = reshape(nums(17:32),4,4)';
    COV  = [];
    else
        T_SICP = [];
        T_GICP = [];
        COV = [];
    end
end

fclose(fileID);
end