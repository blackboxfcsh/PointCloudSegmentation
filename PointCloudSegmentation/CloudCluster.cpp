
#include "CloudCluster.h"

// define the destructor
CloudCluster::~CloudCluster(){

	pointCloudClusterXYZ.reset();
	pointCloudClusterRGB.reset();
	pointCloudClusterNormals.reset();
	//delete cluster_centroid;
}

void CloudCluster::setClusterCentroid(float x, float y, float z){

	cluster_centroid.x = x;
	cluster_centroid.y = y;
	cluster_centroid.z = z;
}

void CloudCluster::SetMaxPointAABB(float x, float y, float z){

	maxPointAABB.x = x;
	maxPointAABB.y = y;
	maxPointAABB.z = z;
}

void CloudCluster::SetMinPointAABB(float x, float y, float z){

	minPointAABB.x = x;
	minPointAABB.y = y;
	minPointAABB.z = z;
}

bool CloudCluster::isCentroidInsideAABB(float x, float y, float z) {

	// check x, y and z
	return (x >= minPointAABB.x && x <= maxPointAABB.x &&
		y >= minPointAABB.y && y <= maxPointAABB.y &&
		z >= minPointAABB.z && z <= maxPointAABB.z);
		
}


bool CloudCluster::areAABBColliding(CloudCluster* currentCluster) {
	PointXYZ minAABB = currentCluster->getMinPointAABB();
	PointXYZ maxAABB = currentCluster->getMaxPointAABB();
	return(maxPointAABB.x > minAABB.x &&
		minPointAABB.x < maxAABB.x &&
		maxPointAABB.y > minAABB.y &&
		minPointAABB.y < maxAABB.y &&
		maxPointAABB.z > minAABB.z &&
		minPointAABB.z < maxAABB.z);
}


void CloudCluster::writeCluster2PCDFile(string filepath, PCDWriter writer) {

	//write cluster data
	stringstream ss;

	ss << filepath << "_cloud_cluster_" << cluster_index << ".pcd";
	writer.write<PointXYZRGB>(ss.str(), *pointCloudClusterRGB, false); //*
}

void CloudCluster::writeCluster2PLYile(string filepath){

	string path = filepath + "_cloud_cluster_" + boost::lexical_cast<std::string>(cluster_index)+".ply";

	//concatenate points, rgb and normals
	pcl::concatenateFields<PointXYZRGB, Normal, PointXYZRGBNormal>(*pointCloudClusterRGB,
		*pointCloudClusterNormals, *pointCloudClusterRGBNormals);
	pcl::io::savePLYFileASCII(path, *pointCloudClusterRGBNormals);
}