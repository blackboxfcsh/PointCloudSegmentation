
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