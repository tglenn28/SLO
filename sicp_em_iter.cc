/* 
This program iterates through a directory of pcd files (selected by the argument '-d'), and calculates a transformation between each pcd and the previous n pcd files. It writes the SICP and GICP transformation matrices to one text file per alignment

Arguments:
-d directory of pcd files to iterate through
-c location of confusion matrix .csv file (entries must either be separated by ' ' or ' ,'
*/ 

#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <string>
#include <dirent.h>
#include <algorithm>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/console/parse.h>

#include <em_icp.h>
#include <gicp.h>
#include <pcl_2_semantic.h>
#include "read_confusion_matrix.h"
#include "kitti_metrics.h"
#include "bootstrap.h"
#include "filter_range.h"

#include <fstream>

std::vector<std::string>
get_pcd_in_dir(std::string dir_name) {

    DIR           *d;
    struct dirent *dir;
    d = opendir(dir_name.c_str());

    std::vector<std::string> pcd_fns;

    if (d) {
        while ((dir = readdir(d)) != NULL) {

            // Check to make sure this is a pcd file match
            if (std::strlen(dir->d_name) >= 4 &&
                    std::strcmp(dir->d_name + std::strlen(dir->d_name)  - 4, ".pcd") == 0) {
                pcd_fns.push_back(dir_name + "/" + std::string(dir->d_name));
            }
        }

        closedir(d);
    }

    return pcd_fns;
}

int
main (int argc, char** argv)
{
    
    std::string strDirectory;
    std::string strGTFile;
    std::string strCMFile;

    std::string strSource;
    std::string strTarget;
/*
    if ( !pcl::console::parse_argument(argc, argv, "-s", strSource) ) {
        std::cout << "Need source file (-s)\n";
        return (-1);
    }
    if ( !pcl::console::parse_argument(argc, argv, "-t", strTarget) ) {
        std::cout << "Need target file (-t)\n";
        return (-1);
    }
*/
    if ( !pcl::console::parse_argument(argc, argv, "-d", strDirectory ) ) {
        std::cout << "Need pcd directory (-d)\n";
        return (-1);
    }
    if ( !pcl::console::parse_argument(argc, argv, "-c", strCMFile ) ) {
        std::cout << "Need confusion matrix (-c)\n";
        return (-1);
    }

    std::string root = strDirectory.substr(0,28);
    std::cout << "root: " << root << std::endl;
    std::string setNum = strDirectory.substr(strDirectory.size() - 11,2);
    std::cout << "setNum: " << setNum << std::endl;
    //std::string sourceNum = strSource.substr(strSource.size() - 10,6);
    //std::string targetNum = strTarget.substr(strTarget.size() - 10,6);
    //std::string outputFilename = root + "transformations/" + setNum + "/" + sourceNum + "_" + targetNum + ".txt";

    //std::ofstream file(outputFilename);


/*
    if ( !pcl::console::parse_argument(argc, argv, "-s", strDirectory) ) {
        std::cout << "Need source directory (-s)\n";
        return (-1);
    }
    if ( !pcl::console::parse_argument(argc, argv, "-t", strGTFile) ) {
        std::cout << "Need ground truth file (-t)\n";
        return (-1);
    }
    if ( !pcl::console::parse_argument(argc, argv, "-m", strCMFile) ) {
        std::cout << "Need ground confusion matrix file (-m)\n";
        return (-1);
    }
*/

    Eigen::Matrix<double, 34, 34> cm = ReadConfusionMatrix<34>(strCMFile);
    //std::cout << "Confusion Matrix:\n" << cm << std::endl;


    std::vector<std::string> pcd_fns = get_pcd_in_dir(strDirectory);
    std::sort(pcd_fns.begin(),pcd_fns.end());

/*
    std::cout << "PCD FILES\n";
    for(std::string s: pcd_fns) {
        std::cout << s << std::endl;
    }
*/

/*
    KittiMetrics semanticICPMetrics(strGTFile);
    KittiMetrics se3GICPMetrics(strGTFile);
    KittiMetrics GICPMetrics(strGTFile);

    KittiMetrics semanticICPMetrics(strGTFile, &foutSICP);
    KittiMetrics se3GICPMetrics(strGTFile, &foutse3GICP);
    KittiMetrics GICPMetrics(strGTFile, &foutGICP);
    KittiMetrics bootstrapMetrics(strGTFile, &foutBootstrap);
*/

    semanticicp::GICP<pcl::PointXYZ> gicpse3temp;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_fns[0], *cloud);
    //gicpse3temp.setTargetCloud(cloud);
    //auto kdtree = gicpse3temp.getTargetKdTree();
    //auto covs = gicpse3temp.getTargetCovariances();

    Sophus::SE3d sicpTranform;
    Sophus::SE3d prevConsecSicpTransfrom;
    int numDigitsSource;
    int numDigitsTarget;
    std::string sourceNum;
    std::string targetNum;
    //for(size_t n = 0; n<(pcd_fns.size()-3); n+=3) 
    for(size_t n = 0; n<pcd_fns.size(); n++)
    {
	numDigitsTarget = std::to_string(n).length();
	targetNum = "";
	for (int z = numDigitsTarget; z < 6; z++)
		targetNum = targetNum + "0";
	targetNum = targetNum + std::to_string(n);

	std::cout << "Cloud#: " << n << std::endl;		

	for(size_t j = 1; j <= 5; j++) 
	{
	    if ((n+j) < pcd_fns.size())
	    {
		numDigitsSource = std::to_string(n + j).length();
		sourceNum = "";
		for (int z = numDigitsSource; z < 6; z++)
			sourceNum = sourceNum + "0";
		sourceNum = sourceNum + std::to_string(n+j);

		std::string outputFilename = root + "transformations/" + setNum + "/" + sourceNum + "_" + targetNum + ".txt";
		//std::cout << "outputFilename: " << outputFilename << std::endl;
    		std::ofstream file(outputFilename);

		size_t indxTarget = n;
		size_t indxSource = indxTarget + j;
		//size_t indxSource = indxTarget + 3;
		std::string strTarget = pcd_fns[indxTarget];
		std::string strSource = pcd_fns[indxSource];
		pcl::PointCloud<pcl::PointXYZL>::Ptr cloudA (new pcl::PointCloud<pcl::PointXYZL>);

		if (pcl::io::loadPCDFile<pcl::PointXYZL> (strSource, *cloudA) == -1) //* load the file
		{
		    PCL_ERROR ("Couldn't read source file\n");
		    return (-1);
		}

		//filterRange(cloudA, 40.0);

		/*
		std::shared_ptr<semanticicp::SemanticPointCloud<pcl::PointXYZ, uint32_t>>
		    semanticA (new semanticicp::SemanticPointCloud<pcl::PointXYZ, uint32_t> ());

		semanticicp::pcl_2_semantic(cloudA, semanticA);
		semanticA->removeSemanticClass( 3 );
		semanticA->removeSemanticClass( 10 );
		semanticA->removeSemanticClass( 11 );
		//cloudA = semanticA->getpclPointCloud();
		*/

		pcl::PointCloud<pcl::PointXYZL>::Ptr cloudB (new pcl::PointCloud<pcl::PointXYZL>);

		if (pcl::io::loadPCDFile<pcl::PointXYZL> (strTarget, *cloudB) == -1) //* load the file
		{
		    PCL_ERROR ("Couldn't read target file\n");
		    return (-1);
		}

		//filterRange(cloudB, 40.0);

		/*
		std::shared_ptr<semanticicp::SemanticPointCloud<pcl::PointXYZ, uint32_t>>
		    semanticB (new semanticicp::SemanticPointCloud<pcl::PointXYZ, uint32_t> ());

		semanticicp::pcl_2_semantic(cloudB, semanticB);
		semanticB->removeSemanticClass( 3 );
		semanticB->removeSemanticClass( 10 );
		semanticB->removeSemanticClass( 11 );
		//cloudB = semanticB->getpclPointCloud();
		*/

		//auto begin = std::chrono::steady_clock::now();
		//Sophus::SE3d initTransform = semanticICPMetrics.getGTtransfrom(n, n+3);
		Eigen::Matrix4d temp = Eigen::Matrix4d::Identity();
		//Bootstrap boot(cloudA, cloudB);
		//Eigen::Matrix4d temp = (boot.align()).cast<double>();
		//Sophus::SE3d initTransform;
		Sophus::SE3d initTransform(temp);
		//auto end = std::chrono::steady_clock::now();
		//int timeInit = std::chrono::duration_cast<std::chrono::seconds>(end-begin).count();
		//std::cout << "Init MSE "
		         // << bootstrapMetrics.evaluate(initTransform, indxTarget, indxSource, timeInit, 0)
		         // << std::endl;

		//semanticicp::EmIterativeClosestPoint<5> emicp;
		semanticicp::EmIterativeClosestPoint<34> emicp;
		//semanticicp::EmIterativeClosestPoint<11> emicp;
		pcl::PointCloud<pcl::PointXYZL>::Ptr
		  finalCloudem( new pcl::PointCloud<pcl::PointXYZL> );

		//begin = std::chrono::steady_clock::now();
		emicp.setSourceCloud(cloudA);
		emicp.setTargetCloud(cloudB);

		emicp.setConfusionMatrix(cm);
			std::streambuf *coutbuf = std::cout.rdbuf(); //save old buf
	    		std::cout.rdbuf(file.rdbuf()); //redirect std::cout to out.txt!
		if (n == 0)
		    emicp.align(finalCloudem, initTransform);
		else if (j == 1)
		    emicp.align(finalCloudem, prevConsecSicpTransfrom);
		else
		    emicp.align(finalCloudem, sicpTranform);
			std::cout.rdbuf(coutbuf); //reset to standard output again

		//end = std::chrono::steady_clock::now();
		//int timeSICP = std::chrono::duration_cast<std::chrono::seconds>(end-begin).count();
	       // std::cout << "Time Multiclass: "
		//        << timeSICP << std::endl;
		//Sophus::SE3d sicpTranform = emicp.getFinalTransFormation();
		sicpTranform = emicp.getFinalTransFormation();
		if (j == 1)
		    prevConsecSicpTransfrom = emicp.getFinalTransFormation();

		//std::cout << "SICP MSE: "
		//          << semanticICPMetrics.evaluate(sicpTranform, indxTarget,
		//                  indxSource, timeSICP, emicp.getOuterIter())
		//          << std::endl;

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudAnoL (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::io::loadPCDFile<pcl::PointXYZ> (strSource, *cloudAnoL);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudBnoL (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::io::loadPCDFile<pcl::PointXYZ> (strTarget, *cloudBnoL);

		pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZL, pcl::PointXYZL> gicp;
		pcl::PointCloud<pcl::PointXYZL> final1;

		//begin = std::chrono::steady_clock::now();
		gicp.setInputCloud(cloudA);
		gicp.setInputTarget(cloudB);
		gicp.setMaxCorrespondenceDistance(1.5);
		gicp.setMaximumIterations(50);
		gicp.align(final1, (initTransform.matrix()).cast<float>());
		//end = std::chrono::steady_clock::now();
		//int timeGICP = std::chrono::duration_cast<std::chrono::seconds>(end-begin).count();
		//std::cout << "Time GICP: "
		//          << timeGICP << std::endl;
		Eigen::Matrix4f mat = gicp.getFinalTransformation();
		//std::cout << "Final GICP Transform\n";
		//std::cout << mat << std::endl;
		
		if(file.is_open())
		{
			file << gicp.getFinalTransformation() << std::endl;
			file.close();
		}
		else std::cout << "Cannot create output file\n";
	
		//std::cout << "GICP transform: \n";
		//std::cout << gicp.getFinalTransformation() << std::endl;

		Sophus::SE3d gicpTransform2 = Sophus::SE3d::fitToSE3(mat.cast<double>());
		//std::cout << "GICP MSE: "
		 //         << GICPMetrics.evaluate(gicpTransform2, indxTarget, indxSource, timeGICP, 1)
		 //         << std::endl;
	    }
	}

    }



    return (0);
}
