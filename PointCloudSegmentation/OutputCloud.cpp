
#include "OutputCloud.h"

// define the destructor
OutputCloud::~OutputCloud() {

	cloudRGB.reset();
	cloudXYZ.reset();

	list<CloudCluster*>::iterator cloudClusterIter;
	for (cloudClusterIter = clusters.begin(); cloudClusterIter != clusters.end(); ++cloudClusterIter)
	{
		CloudCluster* cluster = (*cloudClusterIter);
		delete cluster;
	}
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

	//std::cout << "The XYZ coordinates of the centroid are: ("
	//	<< centroid[0] << ", "
	//	<< centroid[1] << ", "
	//	<< centroid[2] << ")." << std::endl;

	//cout << "translation x = " << translationRotation[0] << " y = " << translationRotation[1] << " z = " << translationRotation[2] << endl;
	//cout << "rotation x = " << translationRotation[3] << " y = " << translationRotation[4] << " z = " << translationRotation[5] << endl;

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
	//printf("\nMethod #3: using a Quaternion\n");
	//std::cout << q.matrix() << std::endl;

	return;
}

void OutputCloud::loadPointCloudNoFormat(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, string pointCloudPath){
	std::ifstream f;
	f.open(pointCloudPath, fstream::in);
	std::string line;
	string delimiter1 = " ";
	double R = 2;


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
	
		point.b = (uint8_t)stoi(pos[3]);
		point.g = (uint8_t)stoi(pos[4]);
		point.r = (uint8_t)stoi(pos[5]);
	/*	int32_t rgb = (point.r << 16) | (point.g << 8) | point.b;
		point.rgb = *(float *)(&rgb); // */


		if (point.x != 0 && point.y != 0 && point.z != 0)
			cloud->points.push_back(point);
		
		pos->clear();
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

void OutputCloud::loadPointClouds(string pointCloudFileName) {

	PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
	loadPointCloudNoFormat(cloud, pointCloudFileName);
	cloudRGB = cloud;
	copyPointCloud<PointXYZRGB, PointXYZ>(*cloudRGB, *cloudXYZ);
	

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
	ec.setClusterTolerance(0.025);
	ec.setMinClusterSize(4000);
	//ec.setMaxClusterSize(30000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloudXYZ);
	ec.extract(cluster_indices);

	createCloudClusters(cluster_indices);

	return;
}

void OutputCloud::createCloudClusters(vector<pcl::PointIndices> cluster_indices) {

	
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_clusterXYZ(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_clusterRGB(new pcl::PointCloud<pcl::PointXYZRGB>);

		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit){
			cloud_clusterXYZ->points.push_back(cloudXYZ->points[*pit]); //*
			cloud_clusterRGB->points.push_back(cloudRGB->points[*pit]);
		}
		cloud_clusterXYZ->width = cloud_clusterRGB->width = cloud_clusterXYZ->points.size();
		cloud_clusterXYZ->height = cloud_clusterRGB->height = 1;
		cloud_clusterXYZ->is_dense = cloud_clusterRGB->is_dense = true;

		fprintf(logFile, "cluster number of points:%d \n", cloud_clusterXYZ->points.size());

		// calculate bounding box
		pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
		feature_extractor.setInputCloud(cloud_clusterXYZ);
		feature_extractor.compute();

		std::vector <float> moment_of_inertia;
		std::vector <float> eccentricity;
		pcl::PointXYZ min_point_AABB;
		pcl::PointXYZ max_point_AABB;

		feature_extractor.getMomentOfInertia(moment_of_inertia);
		feature_extractor.getEccentricity(eccentricity);
		feature_extractor.getAABB(min_point_AABB, max_point_AABB);

		// calculate cluster centroid
		// compute point cloud position
		// Object to store the centroid coordinates.
		Eigen::Vector4f centroid;
		pcl::compute3DCentroid(*cloud_clusterXYZ, centroid);

		CloudCluster *cloudCluster = new CloudCluster();
		cloudCluster->setClusterCentroid(centroid[0], centroid[1], centroid[2]);
		cloudCluster->setPointCloudClusterXYZ(cloud_clusterXYZ);
		cloudCluster->setPointCloudClusterRGB(cloud_clusterRGB);
		cloudCluster->SetMinPointAABB(min_point_AABB.x, min_point_AABB.y, min_point_AABB.z);
		cloudCluster->SetMaxPointAABB(max_point_AABB.x, max_point_AABB.y, max_point_AABB.z);

		clusters.push_back(cloudCluster);
	}

	return;
}

// not being used
list<CloudCluster*> intersetLists(list<CloudCluster*> list1, list<CloudCluster*> list2) {
 
	list<CloudCluster*> result;
	for (list<CloudCluster*>::const_iterator iter1 = list1.begin(); iter1 != list1.end(); ++iter1)
	{
		CloudCluster* cloudCluster1 = (*iter1);
		for (list<CloudCluster*>::const_iterator iter2 = list2.begin(); iter2 != list2.end(); ++iter2)
		{
			CloudCluster* cloudCluster2 = (*iter2);
			if (cloudCluster1->getPointCloudClusterXYZ()->points.size() != cloudCluster2->getPointCloudClusterXYZ()->points.size())
				result.push_back(*iter1);
		}
	}

	return result;

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
		return;
	}
	else {
		int currentClusterIndex = -1;
	//	int maxClusterIndex = 0;
		list<CloudCluster*> temp;
		list<CloudCluster*>::iterator cloudClusterIter;
		
 		for (cloudClusterIter = clusters.begin(); cloudClusterIter != clusters.end(); ++cloudClusterIter)
		{
			float minDist = 100.0;
			CloudCluster* cloudCluster = (*cloudClusterIter);
			for (list<CloudCluster*>::iterator previousClustersIter = previousClusters.begin(); previousClustersIter != previousClusters.end(); ++previousClustersIter)
			{
				CloudCluster *previousCloudCluster = (*previousClustersIter);
				PointXYZ currentCentroid = cloudCluster->getClusterCentroid();
				// calculate distance between current centroid and previous point cloud centroid
				float dist = euclideanDistance(currentCentroid, previousCloudCluster->getClusterCentroid());

				if (dist < minDist){
					minDist = dist;

					// check if the current centroid is also in the AABB of the previous cluster 
					bool isInsidePreviousClusterAABB = previousCloudCluster->isCentroidInsideAABB(currentCentroid.x, currentCentroid.y, currentCentroid.z);
					bool areAABBCollinding = previousCloudCluster->areAABBColliding(cloudCluster);
					if (isInsidePreviousClusterAABB && areAABBCollinding)
						currentClusterIndex = previousCloudCluster->getClusterIndex();
				}
			}
			if (!isClusterXAlreadyDefined(currentClusterIndex) && currentClusterIndex != -1) {
				cloudCluster->setClusterIndex(currentClusterIndex);
				minDist = 100.0;
				currentClusterIndex = -1;

			}
			else
				temp.push_back(cloudCluster);
		}
		//check if there are more clusters in the current list 
		if (!temp.empty()) {
			int maxIdx = getMaxClusterIndex();
			for (list<CloudCluster*>::iterator iter = temp.begin(); iter != temp.end(); ++iter)
			{
				(*iter)->setClusterIndex(++maxIdx);
			}
		}
	} 
	return;
}

double OutputCloud::computeCloudResolution(const pcl::PointCloud<PointXYZ>::ConstPtr &cloud)
{
	double res = 0.0;
	int n_points = 0;
	int nres;
	std::vector<int> indices(2);
	std::vector<float> sqr_distances(2);
	pcl::search::KdTree<PointXYZ> tree;
	tree.setInputCloud(cloud);

	for (size_t i = 0; i < cloud->size(); ++i)
	{
		if (!pcl_isfinite((*cloud)[i].x))
		{
			continue;
		}
		//Considering the second neighbor since the first is the point itself.
		nres = tree.nearestKSearch(i, 2, indices, sqr_distances);
		if (nres == 2)
		{
			res += sqrt(sqr_distances[1]);
			++n_points;
		}
	}
	if (n_points != 0)
	{
		res /= n_points;
	}
	return res;
}

void OutputCloud::estimateClusterNormals(){

	for (list<CloudCluster*>::const_iterator cloudClusterIter = clusters.begin(); cloudClusterIter != clusters.end(); ++cloudClusterIter)
	{
		// Create the normal estimation class, and pass the input dataset to it
		//pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
		ne.setInputCloud((*cloudClusterIter)->getPointCloudClusterXYZ());
		
		// Create an empty kdtree representation, and pass it to the normal estimation object.
		// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
		ne.setSearchMethod(tree);

		// Output datasets
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

		// Use all neighbors in a sphere of radius of 2 times the resolutions of the point cloud
		double resolution = computeCloudResolution((*cloudClusterIter)->getPointCloudClusterXYZ());
		//cout << "resolution = " << resolution * 5 << endl;
		ne.setRadiusSearch(resolution * 5);

		// Compute the features
		ne.compute(*cloud_normals);
		(*cloudClusterIter)->setPointCloudClusterNormals(cloud_normals);
	}
}

void OutputCloud::visualizePointCloudClusters(){

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0.75, 0.75, 0.75);
	int j = 0;
	for (list<CloudCluster*>::const_iterator cloudClusterIter = clusters.begin(); cloudClusterIter != clusters.end(); ++cloudClusterIter)
	{
		cout << "cluster " << j << endl;

		PointCloud<PointXYZRGB>::Ptr pointCloudCluster = (*cloudClusterIter)->getPointCloudClusterRGB();

		PointXYZ minPointAABB = (*cloudClusterIter)->getMinPointAABB();
		PointXYZ maxPointAABB = (*cloudClusterIter)->getMaxPointAABB();

		// visualize clusters
	//	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(pointCloudCluster, (j * 3 * 10), 80, j * 3 * 20);
	//	viewer->addPointCloud<pcl::PointXYZ>(pointCloudCluster, single_color, "sample cloud" + j);

		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(pointCloudCluster);
		viewer->addPointCloud<pcl::PointXYZRGB>(pointCloudCluster, rgb, "sample cloud" + j);
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "sample cloud" + j); (pointCloudCluster);
		viewer->addCube(minPointAABB.x, maxPointAABB.x, minPointAABB.y, maxPointAABB.y, minPointAABB.z, maxPointAABB.z, 1.0, 1.0, 0.0, "AABB" + j);

		//std::cout << "AABB" << j << std::endl;
		//std::cout << "cloud_cluster_" << j << "\t" << "color  = " << (j * 10) << "," << 255 << "," << j * 20 << std::endl;
		
		j++;
	}

	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();

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


void OutputCloud::visualizePointCloudCluster(int clusterIdx, int fileIDX){

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0.75, 0.75, 0.75);
	
	
	CloudCluster* cloudCluster = getCloudClusterX(clusterIdx);
	if (cloudCluster == NULL) {
		cout << "ERROR in visualizePointCloudCluster: no cluster found with index = " << clusterIdx << endl;
		return;
	}

	PointCloud<PointXYZRGB>::Ptr pointCloudCluster = cloudCluster->getPointCloudClusterRGB();

	PointXYZ minPointAABB = cloudCluster->getMinPointAABB();
	PointXYZ maxPointAABB = cloudCluster->getMaxPointAABB();

	// visualize clusters
	//	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(pointCloudCluster, (j * 3 * 10), 80, j * 3 * 20);
	//	viewer->addPointCloud<pcl::PointXYZ>(pointCloudCluster, single_color, "sample cloud" + j);

	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(pointCloudCluster);
	viewer->addPointCloud<pcl::PointXYZRGB>(pointCloudCluster, rgb, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "sample cloud"); (pointCloudCluster);
	viewer->addCube(minPointAABB.x, maxPointAABB.x, minPointAABB.y, maxPointAABB.y, minPointAABB.z, maxPointAABB.z, 1.0, 1.0, 0.0, "AABB");

	//std::cout << "AABB" << j << std::endl;
	//std::cout << "cloud_cluster_" << j << "\t" << "color  = " << (j * 10) << "," << 255 << "," << j * 20 << std::endl;

	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();

	// visualize result
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	
	//string filename = "CAPTURE" + boost::lexical_cast<std::string>(fileIDX)+ ".png";
	//viewer->saveScreenshot(filename);

	// closes the window after pressing 'q'
	viewer->close();

	return;
}

void OutputCloud::visualizePointCloudClustersNormals()
{
	// --------------------------------------------------------
	// -----Open 3D viewer and add point cloud and normals-----
	// --------------------------------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0.75, 0.75, 0.75);
	int j = 0;
	for (list<CloudCluster*>::const_iterator cloudClusterIter = clusters.begin(); cloudClusterIter != clusters.end(); ++cloudClusterIter)
	{
		
		PointCloud<PointXYZRGB>::Ptr pointCloudCluster = (*cloudClusterIter)->getPointCloudClusterRGB();
		PointCloud<pcl::Normal>::Ptr pointCloudCluterNormals = (*cloudClusterIter)->getPointCloudClusterNormals();


		PointXYZ minPointAABB = (*cloudClusterIter)->getMinPointAABB();
		PointXYZ maxPointAABB = (*cloudClusterIter)->getMaxPointAABB();

		// visualize clusters
		//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(pointCloudCluster, (j * 3 * 10), 80, j * 3 * 20);
		//viewer->addPointCloud<pcl::PointXYZ>(pointCloudCluster, single_color, "sample cloud" + j);

		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(pointCloudCluster);
		viewer->addPointCloud<pcl::PointXYZRGB>(pointCloudCluster, rgb, "sample cloud" + j);
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "sample cloud" + j); (pointCloudCluster);
		viewer->addCube(minPointAABB.x, maxPointAABB.x, minPointAABB.y, maxPointAABB.y, minPointAABB.z, maxPointAABB.z, 1.0, 1.0, 0.0, "AABB" + j);
		viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(pointCloudCluster, pointCloudCluterNormals, 10, 0.2, "normals" + j);

		//std::cout << "AABB" << j << std::endl;
		//std::cout << "cloud_cluster_" << j << "\t" << "color  = " << (j * 10) << "," << 255 << "," << j * 20 << std::endl;

		j++;
	}

	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();

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

void OutputCloud::visualizePointCloudClusterNormals(int clusterIdx)
{
	// --------------------------------------------------------
	// -----Open 3D viewer and add point cloud and normals-----
	// --------------------------------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0.75, 0.75, 0.75);

	CloudCluster* cloudCluster = getCloudClusterX(clusterIdx);
	if (cloudCluster == NULL) {
		cout << "ERROR in visualizePointCloudClusterNormals: no cluster found with index = " << clusterIdx << endl;
		return;
	}

	PointCloud<PointXYZRGB>::Ptr pointCloudCluster = cloudCluster->getPointCloudClusterRGB();
	PointCloud<pcl::Normal>::Ptr pointCloudCluterNormals = cloudCluster->getPointCloudClusterNormals();

	PointXYZ minPointAABB = cloudCluster->getMinPointAABB();
	PointXYZ maxPointAABB = cloudCluster->getMaxPointAABB();

	// visualize clusters
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(pointCloudCluster, (j * 3 * 10), 80, j * 3 * 20);
	//viewer->addPointCloud<pcl::PointXYZ>(pointCloudCluster, single_color, "sample cloud" + j);

	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(pointCloudCluster);
	viewer->addPointCloud<pcl::PointXYZRGB>(pointCloudCluster, rgb, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "sample cloud"); (pointCloudCluster);
	viewer->addCube(minPointAABB.x, maxPointAABB.x, minPointAABB.y, maxPointAABB.y, minPointAABB.z, maxPointAABB.z, 1.0, 1.0, 0.0, "AABB");
	viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(pointCloudCluster, pointCloudCluterNormals, 10, 0.2, "normals");

	//std::cout << "AABB" << j << std::endl;
	//std::cout << "cloud_cluster_" << j << "\t" << "color  = " << (j * 10) << "," << 255 << "," << j * 20 << std::endl;

	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();

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

void OutputCloud::writeClusters2PCDFile(string filepath) {
	
	PCDWriter writer;
	for (list<CloudCluster*>::const_iterator cloudClusterIter = clusters.begin(); cloudClusterIter != clusters.end(); cloudClusterIter++)
	{
		CloudCluster* cloudCluster = (*cloudClusterIter);
		cloudCluster->writeCluster2PCDFile(filepath, writer);
	}
	return;
}

void OutputCloud::writeClusters2PLYFile(string filepath) {

	for (list<CloudCluster*>::const_iterator cloudClusterIter = clusters.begin(); cloudClusterIter != clusters.end(); cloudClusterIter++)
	{
		CloudCluster* cloudCluster = (*cloudClusterIter);
		cloudCluster->writeCluster2PLYile(filepath);
	}
	return;
}

bool OutputCloud::isClusterXAlreadyDefined(int index) {


	for (list<CloudCluster*>::const_iterator cloudClusterIter = clusters.begin(); cloudClusterIter != clusters.end(); cloudClusterIter++)
	{
		CloudCluster* cloudCluster = (*cloudClusterIter);
		if (cloudCluster->getClusterIndex() == index)
			return true;
	}

	return false;
}

// returns the max index attributed to the point cloud clusters
int OutputCloud::getMaxClusterIndex(){

	int max = 0;
	for (list<CloudCluster*>::const_iterator cloudClusterIter = clusters.begin(); cloudClusterIter != clusters.end(); cloudClusterIter++)
	{
		CloudCluster* cloudCluster = (*cloudClusterIter);
		if (cloudCluster->getClusterIndex() > max)
			max = cloudCluster->getClusterIndex();
	}

	return max;
}


CloudCluster* OutputCloud::getCloudClusterX(int clusterIdx) {


	for (list<CloudCluster*>::const_iterator cloudClusterIter = clusters.begin(); cloudClusterIter != clusters.end(); cloudClusterIter++)
	{
		CloudCluster* cloudCluster = (*cloudClusterIter);
		if (cloudCluster->getClusterIndex() == index)
			return cloudCluster;
	}
	return NULL;
}


