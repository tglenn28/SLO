#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/console/parse.h>

#include <semantic_point_cloud.h>
#include <semantic_icp.h>
#include <gicp.h>
#include <semantic_viewer.h>
#include <pcl_2_semantic.h>

// Tyler's additions
#include <fstream>


int
main (int argc, char** argv)
{
    std::string strSource;
    std::string strTarget;
    if ( !pcl::console::parse_argument(argc, argv, "-s", strSource) ) {
        std::cout << "Need source file (-s)\n";
        return (-1);
    }
    if ( !pcl::console::parse_argument(argc, argv, "-t", strTarget) ) {
        std::cout << "Need target file (-t)\n";
        return (-1);
    }


    pcl::PointCloud<pcl::PointXYZL>::Ptr cloudA (new pcl::PointCloud<pcl::PointXYZL>);

    if (pcl::io::loadPCDFile<pcl::PointXYZL> (strSource, *cloudA) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file cloudA.pcd \n");
        return (-1);
    }
    std::cout << "Loaded "
              << cloudA->width * cloudA->height
              << " data points from cloudA.pcd with the following fields: "
              << std::endl;

    std::shared_ptr<semanticicp::SemanticPointCloud<pcl::PointXYZ, uint32_t>>
        semanticAfinal (new semanticicp::SemanticPointCloud<pcl::PointXYZ, uint32_t> ());

    semanticicp::pcl_2_semantic(cloudA, semanticAfinal);
    std::shared_ptr<semanticicp::SemanticPointCloud<pcl::PointXYZ, uint32_t>>
        semanticA (new semanticicp::SemanticPointCloud<pcl::PointXYZ, uint32_t> ());

    semanticicp::pcl_2_semantic(cloudA, semanticA);
    
    //semanticA->removeSemanticClass( 13 );
    //semanticA->removeSemanticClass( 10 );
    //semanticA->removeSemanticClass( 11 );
    //semanticA->removeSemanticClass( 15 );
    //semanticA->removeSemanticClass( 16 );
    //semanticA->removeSemanticClass( 18 );

    pcl::PointCloud<pcl::PointXYZL>::Ptr cloudB (new pcl::PointCloud<pcl::PointXYZL>);

    if (pcl::io::loadPCDFile<pcl::PointXYZL> (strTarget, *cloudB) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file cloudB.pcd \n");
        return (-1);
    }
    std::cout << "Loaded "
              << cloudB->width * cloudB->height
              << " data points from cloudB.pcd with the following fields: "
              << std::endl;

    std::shared_ptr<semanticicp::SemanticPointCloud<pcl::PointXYZ, uint32_t>>
        semanticB (new semanticicp::SemanticPointCloud<pcl::PointXYZ, uint32_t> ());

    semanticicp::pcl_2_semantic(cloudB, semanticB);
    
    //semanticB->removeSemanticClass( 13 );
    //semanticB->removeSemanticClass( 10 );
    //semanticB->removeSemanticClass( 11 );
    //semanticB->removeSemanticClass( 15 );
    //semanticB->removeSemanticClass( 16 );
    //semanticB->removeSemanticClass( 18 );

    semanticicp::SemanticIterativeClosestPoint<pcl::PointXYZ, uint32_t> sicp;

	std::string root = strSource.substr(0,28);
	std::string setNum = strSource.substr(strSource.size() - 22,2);
	std::string sourceNum = strSource.substr(strSource.size() - 10,6);
	std::string targetNum = strTarget.substr(strTarget.size() - 10,6);
	std::string outputFilename = root + "transformations/" + setNum + "/" + sourceNum + "_" + targetNum + ".txt";

	//std::cout << "strSource: " << strSource << "\n";
	//std::cout << "root: " << root << "\n";
	//std::cout << "source num: " << sourceNum << "\n";
	//std::cout << "target num: " << targetNum << "\n";
	//std::cout << "set num: " << setNum << "\n";
	std::cout << "output filename: " << outputFilename << "\n";

	ofstream file(outputFilename);

    //auto begin = std::chrono::steady_clock::now();
    sicp.setInputSource(semanticA);
    sicp.setInputTarget(semanticB);
	std::streambuf *coutbuf = std::cout.rdbuf(); //save old buf
    	std::cout.rdbuf(file.rdbuf()); //redirect std::cout to out.txt!
    sicp.align(semanticAfinal);
	std::cout.rdbuf(coutbuf); //reset to standard output again
    //auto end = std::chrono::steady_clock::now();
    //std::cout << "Time Multiclass: "
              //<< std::chrono::duration_cast<std::chrono::seconds>(end-begin).count() << std::endl;


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudAnoL (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ> (strSource, *cloudAnoL);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudBnoL (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ> (strTarget, *cloudBnoL);

    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZL, pcl::PointXYZL> gicp;
    gicp.setInputCloud(cloudA);
    gicp.setInputTarget(cloudB);
    pcl::PointCloud<pcl::PointXYZL>::Ptr final1(new pcl::PointCloud<pcl::PointXYZL>());
    gicp.align(*final1);

    //std::cout << "GICP transform: \n" << gicp.getFinalTransformation() << std::endl;

	

	if(file.is_open())
	{
		file << gicp.getFinalTransformation() << std::endl;
		file.close();
	}
	else std::cout << "Cannot create output file\n";

/*
    std::shared_ptr<semanticicp::SemanticPointCloud<pcl::PointXYZ, uint32_t>>
        gicpFinal (new semanticicp::SemanticPointCloud<pcl::PointXYZ, uint32_t> ());

    semanticicp::pcl_2_semantic(final1, gicpFinal);
    semanticicp::SemanticViewer<pcl::PointXYZ, uint32_t> viewer;
    viewer.addSemanticPointCloudSingleColor( semanticB, 231,41,138, "Target"); //pink
    viewer.addSemanticPointCloudSingleColor( semanticAfinal, 27,158,119, "GICP"); //green
    viewer.addSemanticPointCloudSingleColor( gicpFinal, 217,95,2, "Semantic ICP"); //orange

    while(!viewer.wasStopped()) {
        std::this_thread::sleep_for (std::chrono::microseconds (100000));
    };
*/



    return (0);
}
