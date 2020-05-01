# SLO
Semantic Lidar Odometry

This is the Mobile Robotics Project for the Ford 3 Group: 
__Members__ Ali Badredine, Austin Jeffries, and Tyler Glenn

## SLO



Using the semantic-icp algorithm in conjunction with the widely available GTSAM library we were able to compute the relative transformation between two semantically labelled point clouds that are near and used these calculated transformations to do odometry using the iSAM2 solver in GTSAM. The code implememtation is conatined on this page. 

[![SLO](utils/images/slo_screen_grab.png)](https://www.youtube.com/watch?v=0wvZ5xyvVrM)

## Required Libraries
* GTSAM
* semantic-icp


Our master branch is organized into three main folders. 


## SLAM_ON_LIDAR

This folder contains code for the following:
1. The main run file to ```run_00.m ```
2. The main run file to ``` run_07.m ```

## Semantic-icp

This folder contains our slightly modified version of the great work done by Steven Parkison. Please refrence the read me inside this folder to get this code working. [README.md](https://github.com/tglenn28/SLO/tree/master/Semantic-icp)

## Utils

This folder utility code for our implementation of Semantic Lidar Odometry
Please refrence the read me in this folder to get this code working. [README.md](https://github.com/tglenn28/SLO/tree/master/utils)

## License


## Acknowledgements

We would like to thank our professor, Maani Ghaffari Jadidi for his overall guidance on this project and Steven Parkison for his support in using his SICP software library correctly.



