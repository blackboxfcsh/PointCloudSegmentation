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

// normal estimation
#include <pcl/features/normal_3d_omp.h>

// EuclideanClusterExtraction
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
// calculate point cloud clusters bounding boxes
#include <pcl/features/moment_of_inertia_estimation.h>

using namespace std;
using namespace pcl;

class OutputCloud {

	private:
		int index;

		PointCloud<PointXYZRGB>::Ptr cloudRGB;
		PointCloud<PointXYZ>::Ptr cloudXYZ;

		list<CloudCluster*> clusters;

		FILE* logFile;


	public:

		// constructor(s) and destructor
		OutputCloud() :cloudXYZ(new PointCloud<PointXYZ>) { }
		
		~OutputCloud();


		// accessors
		PointCloud<PointXYZRGB>::Ptr getPointCloudRGB() const { return cloudRGB; }
		//void setPointCloudRGB(PointCloud<PointXYZRGB>::Ptr cloudrgb) { cloudRGB = cloudrgb; }

		PointCloud<PointXYZ>::Ptr getPointCloudXYZ() const { return cloudXYZ; }
		//void setPointCloudXYZ(PointCloud<PointXYZ>::Ptr cloudxyz) { cloudXYZ = cloudxyz; }

		int getIndex() { return index; }
		void setIndex(int idx) { index = idx; }

		list<CloudCluster*> getClusters() const { return clusters; }

		void setLogFile(FILE* file) { logFile = file; }

		// methods
		void applyCalibrationToPointCloud(PointCloud<PointXYZRGB>::Ptr cloud, string calibrarionFilePath);
		void loadPointCloudNoFormat(PointCloud<PointXYZRGB>::Ptr cloud, string pointCloudPath);
		void loadPointClouds(map<string, string> filenameByCalibrationpath);
		void calculatePointCloudClusters();
		void estimateClusterNormals();
		void createCloudClusters(vector<pcl::PointIndices> cluster_indices);
		void determinePointCloudClustersIndex(list<CloudCluster*> previousClusters);
		void visualizePointCloudClusters();
		void visualizePointCloudClustersNormals();

		bool isClusterXAlreadyDefined(int index);
		int getMaxClusterIndex();
		PointCloud<PointXYZRGB>::Ptr getClusterX(int index);

		void writeClusters2PCDFile(string filepath);
		void writeClusters2PLYFile(string filepath);
		

};
