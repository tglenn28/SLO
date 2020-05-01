## SLAM_ON_LIDAR 
# 
(Applying ISAM2 on ICP Transformations With Loopclosures)

This folder contains all files and code needed to succesfully run iSAM2 on GICP and SICP transformations as generated for KITTI data sets 00 and 07, with an additional 07 ICP transforormation where some moving labels were removed!

To run iSAM on 00_map data for both sicp and gicp transformations - (transformation data is already parses from ICP output and in .mat files which are loaded in for the SLAM application):

1. Install GTSAM

2. Clone SLAM_ON_LIDAR repository and add to your matlab path

3. Open run_00.m for data set 00 or run_07.m for data set 07.

3.  Assign path for data (datapath = ...) ```transformations&GT00_new.mat``` (or the other GICP/SICP transformation data files included); these files located in folder ```Trransformation_Data``` already contains the ground truth data 
*the ```.mat``` files were already parsed from the generated sicp gicp transformations using ```.m``` files in folder “Generate_Transformations”.

4. run files from 3.

## Required Libraries
* GTSAM
* semantic-icp (OR generated transformations in this folder can be used)

## Future Work
Additional_Work folder contains attempt on applying concurrent filtering and smoothing using generated loop closures on the ICP transformations. Code is runable, but result is not optimal yet. 

***In the results folders first element in ```Error_iSAM.mat``` corresponds to the GICP error results and second element corresponds to SICP.
