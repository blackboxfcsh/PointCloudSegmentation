
#include "CloudCluster.h"

// define the destructor
CloudCluster::~CloudCluster(){

	//delete cluster_centroid;
}

void CloudCluster::setClusterCentroid(float x, float y, float z){

	cluster_centroid.x = x;
	cluster_centroid.y = y;
	cluster_centroid.z = z;
}

void CloudCluster::writeCluster2File(string filepath) {

	//write cluster data
	pcl::PCDWriter writer;
	std::stringstream ss;
	ss << filepath << "_cloud_cluster_" << cluster_index << ".pcd";
	writer.write<PointXYZ>(ss.str(), *pointCloudCluster, false); //*	
}