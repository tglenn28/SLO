To run iSAM on 00_map data for both sicp and gicp transformations:

1. Open run_00.m
2.  Assign ground truth path for kitti data set. (not necessary if utilizing data in step 3)

AND/OR

3.  Assign path for data “transformations&GT00.mat” ; this file located in folder “Trransformation_Data” already contains the ground truth data (this.mat file was already parsed from the generated sicp gicp transformations using .m files in folder “Generat_Transformations”)

4. Make sure to add all files in folder to matlab path and run the “run_00.m”  to visualize the iSAM results and compare then when using GICP vs SICP transformations.

###################################################################

For 07 data open run_07 and make sure to load "transformations&GT07_labels_removed.mat" OR "transformations&GT07.mat"

###################################################################
