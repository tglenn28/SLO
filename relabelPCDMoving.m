%% Relabel All PCDs with Moving 
clear all; close all; clc;
classes2Remove = [1; %0: "unlabeled"
  2; % 1: "outlier"
%   3; %10%: "car"
%   4; %11%: "bicycle"
%   5; %13%: "bus"
%   6; %15%: "motorcycle"
%   7; %16%: "on-rails"
%   8; %18%: "truck"
%   9; %20%: "other-vehicle"
%   10; %30%: "person"
%   11; %31%: "bicyclist"
%   12; %32%: "motorcyclist"
%   13; %40%: "road"
%   14  %44: "parking"
%   15  %48: "sidewalk"
%   16  %49: "other-ground"
%   17; %50: "building"
%   18; %51: "fence"
%   19; %52: "other-structure"
%   20; %60: "lane-marking"
%   21; %70: "vegetation"
%   22; %71: "trunk"
%   23; %72: "terrain"
%   24; %80: "pole"
%   25; %81: "traffic-sign"
%   26; %99: "other-object"
  27; %252: "moving-car"
  28; %253: "moving-bicyclist"
  29; %254: "moving-person"
  30; %255: "moving-motorcyclist"
  31; %256: "moving-on-rails"
  32; %%257: "moving-bus"
  33; %258: "moving-truck"
  34; ];%259: "moving-other-vehicle"];
%% Specify File Locations
files2Change = dir('/Volumes/DRAGON/KITTI/KITTI_MOD/03_5_connections/velodyne/*.pcd');
newFileDirectory4PCDs = '/Volumes/DRAGON/KITTI/KITTI_MOD/03_5_connections/removedMovingPCDs';
%% Loop Through Files
for iFile=1:numel(files2Change)
    fileLocation = fullfile(files2Change(iFile).folder,files2Change(iFile).name);
    
    newPCDData= relabelPCD(fileLocation, newFileDirectory4PCDs,classes2Remove);
end
function newPCDData = relabelPCD(fileLocation,newFileDirectory, classes2Remove)
% ------------------------------------------------------------------------
%   relabelPCD Function
%   Take in a file location for a .PCD file and return the new PCD cloud in
%   an array.
%   
% ------------------------------------------------------------------------
%   Course: EECS 568
%   Author: A. Jeffries
%   Date: 4/13/2020
% ------------------------------------------------------------------------

% Open the  File
fileID_in = fopen(fileLocation,'r+'); % Open a File for Reading

[~,name,ext] = fileparts(fileLocation);
newFileLocation = fullfile(newFileDirectory,strcat(name,ext)); % Open a File for Writing
fileID_out = fopen(newFileLocation, 'w');


%% Read The Header and Data File and Store
%hdr = strtrim(regexp(fgetl(fileID ),'\t','split'));
pcdData = textscan(fileID_in, '%f %f %f %f %f', 'headerLines', 11, 'CollectOutput', true);
pcdData = cell2mat(pcdData);

conditions2Remove = false(size(pcdData,1),1);
for i=1:numel(classes2Remove)
    conditions2Remove = conditions2Remove | classes2Remove(i) == pcdData(:,5);

end
newPCDData = pcdData(~conditions2Remove,:);

frewind(fileID_in); % Reset file Indicator to teh Begnning of File
% Loop Through Each line of Header
for k=1:11
    this_line =fgets(fileID_in);
    if k == 7 || k == 10
        this_line = regexprep(this_line,'(\d*)', num2str(size(newPCDData,1)));

    end
    fwrite(fileID_out,this_line);
end
% Write All Point Cloud Data
fprintf(fileID_out,'%f %f %f %f %g\n',newPCDData');


%% Close the OrginalFile and Newly Created File
fclose(fileID_in);
fclose(fileID_out);
end

