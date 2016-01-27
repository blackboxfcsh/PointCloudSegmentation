
#include "OutputCloud.h"

// define the destructor
OutputCloud::~OutputCloud() {

	cloudRGB.reset();
	cloudXYZ.reset();

	/*list<CloudCluster*>::iterator cloudClusterIter;
	for (cloudClusterIter = clusters.begin(); cloudClusterIter != clusters.end(); ++cloudClusterIter)
	{
		delete (*cloudClusterIter);
	}*/
}

void OutputCloud::applyCalibrationToPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, string calibrarionFilePath){

	PointCloud<pcl::PointXYZRGB>::Ptr tmp;
	tmp = cloud;
	boost::filesystem::path full_path(boost::filesystem::current_path());

	// Get calibration data
	std::ifstream f;
	f.open(full_path.string() + "\\" + calibrarionFilePath, fstream::in);
	std::string line;
	string delimiter = "=";
	float *translationRotation = new float[6]; // posx, posy, posz, rotx, roty, rotz
	size_t p;
	int i = 0;

	while (std::getline(f, line))
	{
		if ((p = line.find(delimiter)) != std::string::npos) {
			translationRotation[i] = stof(line.substr(p + 1, line.length()));
			i++;
		}
	}

	// compute point cloud position
	// Object to store the centroid coordinates.
	Eigen::Vector4f centroid;

	pcl::compute3DCentroid(*cloud, centroid);

	std::cout << "The XYZ coordinates of the centroid are: ("
		<< centroid[0] << ", "
		<< centroid[1] << ", "
		<< centroid[2] << ")." << std::endl;

	cout << "translation x = " << translationRotation[0] << " y = " << translationRotation[1] << " z = " << translationRotation[2] << endl;
	cout << "rotation x = " << translationRotation[3] << " y = " << translationRotation[4] << " z = " << translationRotation[5] << endl;

	float thetaX = deg2rad(translationRotation[3]); // The angle of rotation in radians
	float thetaY = deg2rad(translationRotation[4]);; // The angle of rotation in radians
	float thetaZ = deg2rad(translationRotation[5]);; // The angle of rotation in radians 

	// setting up the quaternion and the translation
	Eigen::AngleAxisd rollAngle(thetaZ, Eigen::Vector3d::UnitZ());
	Eigen::AngleAxisd yawAngle(thetaY, Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd pitchAngle(thetaX, Eigen::Vector3d::UnitX());
	Eigen::Vector3d translate(translationRotation[0], translationRotation[1], translationRotation[2]);
	Eigen::Quaterniond q = rollAngle * yawAngle * pitchAngle;

	pcl::transformPointCloud(*cloud, *cloud, translate, q);

	cloud = tmp;

	// Print the transformation
	printf("\nMethod #3: using a Quaternion\n");
	std::cout << q.matrix() << std::endl;

	return;
}

void OutputCloud::loadPointCloudNoFormat(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, string pointCloudPath){
	std::ifstream f;
	f.open(pointCloudPath, fstream::in);
	std::string line;
	string delimiter1 = " ";

	string *pos = new string[6];
	while (std::getline(f, line))
	{

		size_t p;
		int i = 0;
		string substring = "";
		while ((p = line.find(delimiter1)) != std::string::npos) {
			substring = pos[i] = line.substr(0, p);
			line.erase(0, p + delimiter1.length());
			i++;
		}
		pos[5] = line;

		pcl::PointXYZRGB point;
		point.x = stof(pos[0]);
		point.y = stof(pos[1]);
		point.z = stof(pos[2]);
		point.r = (uint8_t)stoi(pos[3]);
		point.g = (uint8_t)stoi(pos[4]);
		point.b = (uint8_t)stoi(pos[5]);

		if (point.x != 0 && point.y != 0 && point.z != 0)
			cloud->points.push_back(point);
	}
}

void OutputCloud::loadPointClouds(map<string, string> filenameByCalibrationpath) {

	PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempCloud(new PointCloud<PointXYZRGB>);
	map<string, string>::iterator iter;
	for (iter = filenameByCalibrationpath.begin(); iter != filenameByCalibrationpath.end(); iter++) {
		loadPointCloudNoFormat(cloud, iter->first);
		applyCalibrationToPointCloud(cloud, iter->second);
		*tempCloud += *cloud;
		cloud->clear();
	}
	cloudRGB = tempCloud;
}

void OutputCloud::calculatePointCloudClusters() {

	EuclideanClusterExtraction<PointXYZ> ec;
	// Creating the KdTree object for the search method of the extraction
	search::KdTree<pcl::PointXYZ>::Ptr tree(new search::KdTree<PointXYZ>);

	//create XYZ point cloud from RGB cloud

	copyPointCloud<PointXYZRGB, PointXYZ>(*cloudRGB, *cloudXYZ);
	tree->setInputCloud(cloudXYZ);

	std::vector<pcl::PointIndices> cluster_indices;
	//ec.setClusterTolerance(0.028); // 2,8cm 
	ec.setClusterTolerance(0.028);
	ec.setMinClusterSize(4000);
	//ec.setMaxClusterSize(40000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloudXYZ);
	ec.extract(cluster_indices);

	createCloudClusters(cluster_indices);

	return;
}

void OutputCloud::createCloudClusters(vector<pcl::PointIndices> cluster_indices) {

	
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
			cloud_cluster->points.push_back(cloudXYZ->points[*pit]); //*
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		// calculate cluster centroid
		// compute point cloud position
		// Object to store the centroid coordinates.
		Eigen::Vector4f centroid;
		pcl::compute3DCentroid(*cloud_cluster, centroid);

		std::cout << "The XYZ coordinates of the centroid are: ("
			<< centroid[0] << ", " 
			<< centroid[1] << ", "
			<< centroid[2] << ")." << std::endl;

		CloudCluster *cloudCluster = new CloudCluster();
		cloudCluster->setClusterCentroid(centroid[0], centroid[1], centroid[2]);
		cloudCluster->setPointCloudCluster(cloud_cluster);

		clusters.push_back(cloudCluster);
	}

	return;
}


void OutputCloud::determinePointCloudClustersIndex(list<CloudCluster*> previousClusters) {

	if (previousClusters.empty()) {
		int i = 0;
		for (list<CloudCluster*>::iterator cloudClusterIter = clusters.begin(); cloudClusterIter != clusters.end(); ++cloudClusterIter)
		{
			CloudCluster *cloudCluster = (*cloudClusterIter);
			cloudCluster->setClusterIndex(i);
			i++;
		}
	}
	else {
		int currentClusterIndex = 0;
		list<CloudCluster*>::iterator cloudClusterIter;
		for (cloudClusterIter = clusters.begin(); cloudClusterIter != clusters.end(); ++cloudClusterIter)
		{
			float minDist = 100.0;
			CloudCluster* cloudCluster = (*cloudClusterIter);
			for (list<CloudCluster*>::iterator previousClustersIter = previousClusters.begin(); previousClustersIter != previousClusters.end(); ++previousClustersIter)
			{
				CloudCluster *previousCloudCluster = (*previousClustersIter);
				float dist = euclideanDistance(cloudCluster->getClusterCentroid(), previousCloudCluster->getClusterCentroid());
				if (dist < minDist) {
					minDist = dist;
					currentClusterIndex = previousCloudCluster->getClusterIndex();
				}
			}
			cloudCluster->setClusterIndex(currentClusterIndex);
		}
		//check if there are more clusters in the current list 
		while (cloudClusterIter != clusters.end()) {
			CloudCluster* cloudCluster = (*cloudClusterIter);
			cloudCluster->setClusterIndex(++currentClusterIndex);
			cloudClusterIter++;
		}
	} 
	return;
}

void OutputCloud::visualizePointCloudClusters(){

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	int j = 0;
	for (list<CloudCluster*>::const_iterator cloudClusterIter = clusters.begin(); cloudClusterIter != clusters.end(); ++cloudClusterIter)
	{
		cout << "cluster " << j << endl;

		PointCloud<PointXYZ>::Ptr pointCloudCluster = (*cloudClusterIter)->getPointCloudCluster();

		// calculate bounding box
		pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
		feature_extractor.setInputCloud(pointCloudCluster);
		feature_extractor.compute();

		std::vector <float> moment_of_inertia;
		std::vector <float> eccentricity;
		pcl::PointXYZ min_point_AABB;
		pcl::PointXYZ max_point_AABB;

		feature_extractor.getMomentOfInertia(moment_of_inertia);
		feature_extractor.getEccentricity(eccentricity);
		feature_extractor.getAABB(min_point_AABB, max_point_AABB);

		// visualize clusters
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(pointCloudCluster, (j * 3 * 10), 80, j * 3 * 20);
		viewer->addPointCloud<pcl::PointXYZ>(pointCloudCluster, single_color, "sample cloud" + j);
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud" + j); (pointCloudCluster);
		viewer->addCube(min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0, 0.0, "AABB" + j);

		std::cout << "AABB" << j << std::endl;
		std::cout << "cloud_cluster_" << j << "\t" << "color  = " << (j * 10) << "," << 255 << "," << j * 20 << std::endl;
		
		j++;
	}

	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();

	//viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);
	//viewer->registerMouseCallback(mouseEventOccurred, (void*)&viewer);


	// visualize result
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	// closes the window after pressing 'q'
	viewer->close();

	return;
}

void OutputCloud::writeClusters2File(string filepath) {
	
	PCDWriter writer;
	for (list<CloudCluster*>::const_iterator cloudClusterIter = clusters.begin(); cloudClusterIter != clusters.end(); ++cloudClusterIter)
	{
		CloudCluster* cloudCluster = (*cloudClusterIter);
		cloudCluster->writeCluster2File(filepath, writer);
	}
	return;
}

PointCloud<PointXYZ>::Ptr OutputCloud::getClusterX(int index) {

	for (list<CloudCluster*>::const_iterator cloudClusterIter = clusters.begin(); cloudClusterIter != clusters.end(); ++cloudClusterIter)
	{
		CloudCluster* cloudCluster = (*cloudClusterIter);
		if (cloudCluster->getClusterIndex() == index)
			return cloudCluster->getPointCloudCluster();
	}
}