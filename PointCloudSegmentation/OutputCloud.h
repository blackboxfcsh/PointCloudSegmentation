#define NOMINMAX
#define _CRT_SECURE_NO_WARNINGS

#include "CloudCluster.h"

#include <Windows.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>


using namespace std;
using namespace pcl;

class OutputCloud {

	private:
		int index;

		PointCloud<pcl::PointXYZRGB>::Ptr cloudRGB;
		PointCloud<pcl::PointXYZ> cloudXYZ;

		list<CloudCluster> clusters;

	public:

		// constructor(s) and destructor
		OutputCloud() {}
		
		/*OutputCloud(PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb) 
			:cloudRGB(cloudrgb) {

			copyPointCloud<PointXYZRGB, PointXYZ>(*cloudrgb, *cloudXYZ);
		}*/
		

		// accessors
		PointCloud<PointXYZRGB>::Ptr getPointCloudRGB() const { return cloudRGB; }
		/*void setPointCloudRGB(PointCloud<PointXYZRGB>::Ptr cloudrgb) { cloudRGB = cloudrgb; }

		PointCloud<PointXYZ>::Ptr getPointCloudXYZ() const { return cloudXYZ; }
		void setPointCloudXYZ(PointCloud<PointXYZ>::Ptr cloudxyz) { cloudXYZ = cloudxyz; }*/

		void setIndex(int idx) { index = idx; }

		// methods
		PointCloud<pcl::PointXYZRGB>::Ptr ApplyCalibrationToPointCloud(PointCloud<PointXYZRGB>::Ptr cloud, string calibrarionFilePath);
		void loadPointCloudNoFormat(PointCloud<PointXYZRGB>::Ptr cloud, string pointCloudPath);
		void loadPointClouds(map<string, string> filenameByCalibrationpath);
		
		void WriteClusters2File();
		

};
