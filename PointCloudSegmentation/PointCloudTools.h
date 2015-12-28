#define NOMINMAX
#define _CRT_SECURE_NO_WARNINGS

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

// Euclidean Cluster Extraction
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>

using namespace std;
namespace pctools {

	class PointCloudTools {

	public:

		/**
		Loads a point cloud from a file
		*/
		void loadPointCloudNoFormat(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, string pointCloudPath);

		/**
		Applies a translation followed by a rotation: calibrateas the point cloud according the camera viewpoint
		*/
		void ApplyCalibrationToPointCloud(list<pcl::PointCloud<pcl::PointXYZ>> cloud, string calibrarionFilePath);
	
		/**
		Segments a point cloud according to the euclidean distance of points
		*/
		std::vector<pcl::PointIndices> EuclideanClusterExtractionSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ);

	private:
		
		pcl::PointCloud<pcl::PointXYZRGB> cloud;


	
	};
}