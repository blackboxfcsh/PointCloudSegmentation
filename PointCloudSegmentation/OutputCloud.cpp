
#include "OutputCloud.h"

pcl::PointCloud<pcl::PointXYZRGB>::Ptr OutputCloud::ApplyCalibrationToPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, string calibrarionFilePath){

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	boost::filesystem::path full_path(boost::filesystem::current_path());

	// Get calibration data
	std::ifstream f;
	f.open(full_path.string() + "\\" + calibrarionFilePath, fstream::in);
	std::string line;
	string delimiter = "=";
	float *translationRotation = new float[6]; // posx, posy, posz, rotx, roty, rotz
	size_t p;
	int i = 0;
	//string substring = "";

	while (std::getline(f, line))
	{
		if ((p = line.find(delimiter)) != std::string::npos) {
			translationRotation[i] = stof(line.substr(p + 1, line.length()));
			i++;
		}
		//pos[5] = line.substr(p+1, line.length());
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


	Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

	// translate to 0
	// then rotate 
	// then translate to initial position

	// Define a translation to 0,0,0.
	//transform_2.translation() << translationRotation[0], translationRotation[1], translationRotation[2];
	transform_2.translation() << -centroid[0], -centroid[1], -centroid[2];

	/*float thetaX = (M_PI / 180) * translationRotation[3]; // The angle of rotation in radians
	float thetaY = (M_PI / 180) * translationRotation[4]; // The angle of rotation in radians
	float thetaZ = (M_PI / 180) * translationRotation[5]; // The angle of rotation in radians */

	float thetaX = deg2rad(translationRotation[3]); // The angle of rotation in radians
	float thetaY = deg2rad(translationRotation[4]);; // The angle of rotation in radians
	float thetaZ = deg2rad(translationRotation[5]);; // The angle of rotation in radians 
	// The  rotation matrix; tetha radians arround X, Y and Z axis
	transform_2.rotate(Eigen::AngleAxisf(thetaX, Eigen::Vector3f::UnitX()));
	transform_2.rotate(Eigen::AngleAxisf(thetaY, Eigen::Vector3f::UnitY()));
	transform_2.rotate(Eigen::AngleAxisf(thetaZ, Eigen::Vector3f::UnitZ()));

	cout << "rotation in radians x = " << thetaX << " y = " << thetaY << " z = " << thetaZ << endl;

	transform_2.translation() << centroid[0], centroid[1], centroid[2];

	transform_2.translation() << translationRotation[0], translationRotation[1], translationRotation[2]; \

	// Apply translate to origin, rotation, translate no original position, translate to calibration position
	pcl::transformPointCloud(*cloud, *transformed_cloud, transform_2);

	// Print the transformation
	printf("\nMethod #2: using an Affine3f\n");
	std::cout << transform_2.matrix() << std::endl;

	return transformed_cloud;
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
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedCloud;
	map<string, string>::iterator iter;
	for (iter = filenameByCalibrationpath.begin(); iter != filenameByCalibrationpath.end(); iter++) {
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr tempCloud;
		loadPointCloudNoFormat(cloud, iter->first);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformedCloud = ApplyCalibrationToPointCloud(cloud, iter->second);
		*cloud += *transformedCloud;
	}
	cloudRGB = cloud;
}