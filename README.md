# SLO
Semantic Lidar Odometry

This is the Mobile Robotics Project for the Ford 3 Group: 
__Members__ Ali Badredine, Austin Jeffries, and Tyler Glenn

## SLO

Using the semantic-icp algorithm in conjunction with the widely available GTSAM library we were able to first compute the relative transformation between two point clouds that are near - both consecutive and ones that are separated between 1 and 5 point clouds away and used these calculated transformations to do odometry using the iSAM2 solver in GTSAM. As a comparison, we also calculated transformations using the generalized iterative closest point algorithm as a baseline. Please see our video below for a brief explanation of our project.

[![SLO](https://github.com/tglenn28/SLO/tree/master/utils/images/slo_screen_grab.png)](https://www.youtube.com/watch?v=0wvZ5xyvVrM&t=2s)

Our master branch is organized into three main folders. 


## SLAM_ON_LIDAR

This folder contains code for the following:
1. The main run file to run iSAM2 solver
2. Our Matlab Visualization tool

## Semantic-icp

This folder contains our slightly modified version of the great work done by Steven Parkison. Please refrence the read me inside this folder to get this code working. [README.md](https://github.com/tglenn28/SLO/tree/master/Semantic-icp)

## Utils

This folder utility code for our implementation of Semantic Lidar Odometry
Please refrence the read me in this folder to get this code working. [README.md](https://github.com/tglenn28/SLO/tree/master/utils)


