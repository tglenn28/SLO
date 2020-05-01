This folder contains all files and code needed to succesfully run iSAM2 on GICP and SICP transformations as generated for KITTI data sets 00 and 07, with an additional 07 ICP transforormation where some moving labels were removed!


To run iSAM on 00_map data for both sicp and gicp transformations - (transformation data is already parses from ICP output and in .mat files which are loaded in for the SLAM application):

1. Open run_00.m

3.  Assign path for data “transformations&GT00_new.mat” ; this file located in folder “Trransformation_Data” already contains the ground truth data (this.mat file was already parsed from the generated sicp gicp transformations using .m files in folder “Generat_Transformations”)

4. Make sure to add all files in folder to matlab path and run the “run_00.m”  to visualize the iSAM results and compare then when using GICP vs SICP transformations.

###################################################################

For 07 data open run_07 and make sure to load "transformations&GT07_labels_removed.mat" OR "transformations&GT07.mat"

###################################################################

***GTSAM is needed to run this.
***ADDITIONAL_WORK Folder contains application attempt for concurrent filtering and smoothing using generated loop closures and the ICP transformations.
