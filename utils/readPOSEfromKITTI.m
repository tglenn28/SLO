function T = readPOSEfromKITTI(fileLocation)
% ------------------------------------------------------------------------
%   readPOSEfromKITTI Function
%   Take in a file location for a .txt file with 3D POSE Data and return the
%   array
% ------------------------------------------------------------------------
%   Course: EECS 568
%   Author: A. Jeffries
%   Date: 4/13/2020
% ------------------------------------------------------------------------

% Open the  File
fileID = fopen(fileLocation,'r');
% Create Format Specs to Read through the file
formatSpec = ['%f %f %f %f %f %f %f %f %f %f %f %f\n'];

%% Read The File and Store in a matrix
A = fscanf(fileID,formatSpec); % Read in the Matrix
N = size(A,1);
n = 12;
T = reshape(A,n,N/n)';
T = [(0:size(T,1)-1)' T]; % Ad Pose Number

%% Close the File
fclose(fileID);

end

