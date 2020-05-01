#!/bin/bash

PROJDIR=/home/tyler/rob530proj
POINTCLOUDDIR="${PROJDIR}/data/point_clouds/00"
PCDDIR="${PROJDIR}/data/pcds/"
LABELEDPCDDIR="${PROJDIR}/data/pcds/"
ADDLABELEXEDIR="${PROJDIR}/source"
KITTI2PCLDIR="${PROJDIR}/kitti2pcl"
VEL=/velodyne

cd $PROJDIR

for f in $(find $POINTCLOUDDIR -name '0000**.bin'); do 
cd $( dirname $f)
ParentDir=$(cd ..; basename -- "$PWD")
cd $PROJDIR
./velo2pcd -a -s $f -o $PCDDIR$ParentDir$VEL; done

echo $f
#echo $ParentDir

cd $PROJDIR

for f in $(find $PCDDIR -name '000***.pcd'); do ./addLabelsToPCDSequential $f 8; done
