/* \author Geoffrey Biggs */
#define NOMINMAX
#define _CRT_SECURE_NO_WARNINGS

// ALREADY IN OUTPUTCLOUD.H
#include <Windows.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>
#include <map>

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
#include <pcl/features/moment_of_inertia_estimation.h>

// conditional segmentation

#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>

//END ALREADY IN OUTPUTCLOUD.H

// Euclidean Cluster Extraction
//#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/segmentation/extract_clusters.h>
//#include <pcl/filters/voxel_grid.h>

// region growing segmentation
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/segmentation/region_growing_rgb.h>


#include <pcl/common/time.h>
#include <pcl/common/centroid.h>

#include <boost/format.hpp>

#include "Node.h"
//#include "OutputCloud.h"

using namespace std;
using namespace pcl;

//Globals

// --------------
// -----Help-----
// --------------
void
printUsage(const char* progName)
{
	std::cout << "\n\nUsage: " << progName << " [options]\n\n"
		<< "Options:\n"
		<< "-------------------------------------------\n"
		<< "-h           this help\n"
		<< "-s           Simple visualisation example\n"
		<< "-r           RGB colour visualisation example\n"
		<< "-c           Custom colour visualisation example\n"
		<< "-n           Normals visualisation example\n"
		<< "-a           Shapes visualisation example\n"
		<< "-v           Viewports example\n"
		<< "-i           Interaction Customization example\n"
		<< "\n\n";
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
	// --------------------------------------------
	// -----Open 3D viewer and add point cloud-----
	// --------------------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	return (viewer);
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
	// --------------------------------------------
	// -----Open 3D viewer and add point cloud-----
	// --------------------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	return (viewer);
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> customColourVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
	// --------------------------------------------
	// -----Open 3D viewer and add point cloud-----
	// --------------------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, single_color, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	return (viewer);
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis(
	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals)
{
	// --------------------------------------------------------
	// -----Open 3D viewer and add point cloud and normals-----
	// --------------------------------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, normals, 10, 0.05, "normals");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	return (viewer);
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> shapesVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
	// --------------------------------------------
	// -----Open 3D viewer and add point cloud-----
	// --------------------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();

	//------------------------------------
	//-----Add shapes at cloud points-----
	//------------------------------------
	viewer->addLine<pcl::PointXYZRGB>(cloud->points[0],
		cloud->points[cloud->size() - 1], "line");
	viewer->addSphere(cloud->points[0], 0.2, 0.5, 0.5, 0.0, "sphere");

	//---------------------------------------
	//-----Add shapes at other locations-----
	//---------------------------------------
	pcl::ModelCoefficients coeffs;
	coeffs.values.push_back(0.0);
	coeffs.values.push_back(0.0);
	coeffs.values.push_back(1.0);
	coeffs.values.push_back(0.0);
	viewer->addPlane(coeffs, "plane");
	coeffs.values.clear();
	coeffs.values.push_back(0.3);
	coeffs.values.push_back(0.3);
	coeffs.values.push_back(0.0);
	coeffs.values.push_back(0.0);
	coeffs.values.push_back(1.0);
	coeffs.values.push_back(0.0);
	coeffs.values.push_back(5.0);
	viewer->addCone(coeffs, "cone");

	return (viewer);
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> viewportsVis(
	pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals1, pcl::PointCloud<pcl::Normal>::ConstPtr normals2)
{
	// --------------------------------------------------------
	// -----Open 3D viewer and add point cloud and normals-----
	// --------------------------------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->initCameraParameters();

	int v1(0);
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->setBackgroundColor(0, 0, 0, v1);
	viewer->addText("Radius: 0.01", 10, 10, "v1 text", v1);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud1", v1);

	int v2(0);
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer->setBackgroundColor(0.3, 0.3, 0.3, v2);
	viewer->addText("Radius: 0.1", 10, 10, "v2 text", v2);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(cloud, 0, 255, 0);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, single_color, "sample cloud2", v2);

	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud2");
	viewer->addCoordinateSystem(1.0);

	viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, normals1, 10, 0.05, "normals1", v1);
	viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(cloud, normals2, 10, 0.05, "normals2", v2);

	return (viewer);
}


unsigned int text_id = 0;
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event,
	void* viewer_void)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
	if (event.getKeySym() == "r" && event.keyDown())
	{
		std::cout << "r was pressed => removing all text" << std::endl;

		char str[512];
		for (unsigned int i = 0; i < text_id; ++i)
		{
			sprintf(str, "text#%03d", i);
			viewer->removeShape(str);
		}
		text_id = 0;
	} 
	else if (event.getKeySym() == "w" && event.keyDown())
	{
		std::cout << "w was pressed => moving closer" << std::endl;

		//viewer->setCameraPosition(cam[0].pos[0] + 0.2, cam[0].pos[1], cam[0].pos[2], cam[0].view[0], cam[0].view[1], cam[0].view[2], 0);
	}
}

void mouseEventOccurred(const pcl::visualization::MouseEvent &event,
	void* viewer_void)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
	if (event.getButton() == pcl::visualization::MouseEvent::LeftButton &&
		event.getType() == pcl::visualization::MouseEvent::MouseButtonRelease)
	{
		
		std::cout << "Left mouse button released at position (" << event.getX() << ", " << event.getY() << ")" << std::endl;

		char str[512];
		sprintf(str, "text#%03d", text_id++);
		viewer->addText("clicked here", event.getX(), event.getY(), str);
	}
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> interactionCustomizationVis()
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addCoordinateSystem(1.0);

	viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);
	viewer->registerMouseCallback(mouseEventOccurred, (void*)&viewer);

	return (viewer);
}

void visualizePointCloudClusters(std::vector<pcl::PointIndices> cluster_indices, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ){

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit){
			cloud_cluster->points.push_back(cloudXYZ->points[*pit]); 
		}
		
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		cout << "cluster " << j << endl;

		// calculate bounding box
		pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
		feature_extractor.setInputCloud(cloud_cluster);
		feature_extractor.compute();

		std::vector <float> moment_of_inertia;
		std::vector <float> eccentricity;
		pcl::PointXYZ min_point_AABB;
		pcl::PointXYZ max_point_AABB;

		feature_extractor.getMomentOfInertia(moment_of_inertia);
		feature_extractor.getEccentricity(eccentricity);
		feature_extractor.getAABB(min_point_AABB, max_point_AABB);



		// visualize clusters
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud_cluster, (j * 3 * 10), 80, j * 3 * 20);
		viewer->addPointCloud<pcl::PointXYZ>(cloud_cluster, single_color, "sample cloud" + j);
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud" + j); (cloud_cluster);
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

void visualizePointCloudClusters(OutputCloud* outputCloud, int frame){

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Viewer 3D"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addCoordinateSystem(1.0);
	char str[512];
	sprintf(str, "FRAME%i", frame);
	viewer->addText(str, 100, 100);

	// TODO: ADD BOUNDING BOX
	int i = 0;
	list<CloudCluster*> clusters = outputCloud->getClusters();
	cout << "number of clusters = " << clusters.size() << endl;
	for each (CloudCluster* cluster in clusters)
	{
		PointCloud<PointXYZ>::Ptr cloud = cluster->getPointCloudClusterXYZ();

		// calculate bounding box
		pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
		feature_extractor.setInputCloud(cloud);
		feature_extractor.compute();

		std::vector <float> moment_of_inertia;
		std::vector <float> eccentricity;
		pcl::PointXYZ min_point_AABB;
		pcl::PointXYZ max_point_AABB;

		feature_extractor.getMomentOfInertia(moment_of_inertia);
		feature_extractor.getEccentricity(eccentricity);
		feature_extractor.getAABB(min_point_AABB, max_point_AABB);

		
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, (i * 3 * 10), 80, i * 3 * 20);
		viewer->addPointCloud<pcl::PointXYZ>(cloud, single_color, "sample cloud" + i);

		//pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
		//viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud" + i); (cloud);
		viewer->addCube(min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0, 0.0, "AABB" + i);

		i++;
	}

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

void visualizePointCloudCluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int clusterID){

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Viewer 3D"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addCoordinateSystem(1.0);
	char str[512];
	sprintf(str, "CLUSTER%i", clusterID);
	viewer->addText(str, 100, 100);

	// TODO: ADD BOUNDING BOX
	
	// visualize clusters
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 30, 80, 3 * 20);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, single_color, "sample cloud");
	//pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	//viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
	//viewer->addPointCloud(cloud);
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud"); (cloud);
	//viewer->addCube(min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0, 0.0, "AABB");

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

void visualizePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Viewer 3D"));
	viewer->setBackgroundColor(0.75, 0.75, 0.75);
	viewer->addCoordinateSystem(1.0);

	// visualize clusters
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
	viewer->addPointCloud(cloud);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "sample cloud"); (cloud);

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

void pclVisualizer(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){

	bool simple(false), rgb(false), custom_c(false), normals(false),
		shapes(false), viewports(false), interaction_customization(false);
	
	// ------------------------------------
	// -----Create example point cloud-----
	// ------------------------------------
	pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	std::cout << "Genarating example point clouds.\n\n";
	// We're going to make an ellipse extruded along the z-axis. The colour for
	// the XYZRGB cloud will gradually go from red to green to blue.
	uint8_t r(255), g(15), b(15);
	for (float z(-1.0); z <= 1.0; z += 0.05)
	{
	for (float angle(0.0); angle <= 360.0; angle += 5.0)
	{
	pcl::PointXYZ basic_point;
	basic_point.x = 0.5 * cosf(pcl::deg2rad(angle));
	basic_point.y = sinf(pcl::deg2rad(angle));
	basic_point.z = z;
	basic_cloud_ptr->points.push_back(basic_point);

	pcl::PointXYZRGB point;
	point.x = basic_point.x;
	point.y = basic_point.y;
	point.z = basic_point.z;
	uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
	static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
	point.rgb = *reinterpret_cast<float*>(&rgb);
	point_cloud_ptr->points.push_back(point);
	}
	if (z < 0.0)
	{
	r -= 12;
	g += 12;
	}
	else
	{
	g -= 12;
	b += 12;
	}
	}
	basic_cloud_ptr->width = (int)basic_cloud_ptr->points.size();
	basic_cloud_ptr->height = 1;
	point_cloud_ptr->width = (int)point_cloud_ptr->points.size();
	point_cloud_ptr->height = 1;


	// LOAD STANDFORD BUNNY
	pcl::PolygonMesh mesh;
	pcl::io::loadPolygonFilePLY("D:\\PointCloudSegmentation\\bun_love.ply", mesh);
	fromPCLPointCloud2(mesh.cloud, *point_cloud_ptr);

	// ----------------------------------------------------------------
	// -----Calculate surface normals with a search radius of 0.05-----
	// ----------------------------------------------------------------
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	ne.setInputCloud(point_cloud_ptr);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
	ne.setSearchMethod(tree);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1(new pcl::PointCloud<pcl::Normal>);
	ne.setRadiusSearch(0.0005);
	ne.compute(*cloud_normals1);

	// ---------------------------------------------------------------
	// -----Calculate surface normals with a search radius of 0.1-----
	// ---------------------------------------------------------------
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);
	ne.setRadiusSearch(0.1);
	ne.compute(*cloud_normals2);
	
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	if (simple)
	{
		viewer = simpleVis(basic_cloud_ptr);
	}
	else if (rgb)
	{
		viewer = rgbVis(point_cloud_ptr);
	}
	else if (custom_c)
	{
		viewer = customColourVis(basic_cloud_ptr);
	}
	else if (normals)
	{
		viewer = normalsVis(point_cloud_ptr, cloud_normals1);
	}
	else if (shapes)
	{
		viewer = shapesVis(point_cloud_ptr);
	}
	else if (viewports)
	{
		//	viewer = viewportsVis(point_cloud_ptr, cloud_normals1, cloud_normals2);
	}
	else if (interaction_customization)
	{
		viewer = interactionCustomizationVis();
	}

	//--------------------
	// -----Main loop-----
	//--------------------
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

}

// ----------------------------------------------------------------------
// -----LOAD POINT CLOUD DATA--------------------------------------------
// ----------------------------------------------------------------------

vector<string> getFilesInDirectory(const string directory)
{
	HANDLE dir;
	WIN32_FIND_DATA file_data;
	vector<string> out;
	string direc = directory + "/*";
	const char* temp = direc.c_str();

	if ((dir = FindFirstFile(temp, &file_data)) == INVALID_HANDLE_VALUE)
		return out;

	do {
		const string file_name = file_data.cFileName;
		const string full_file_name = directory + "\\" + file_name;
		const bool is_directory = (file_data.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) != 0;

		if (file_name[0] == '.')
			continue;

		if (is_directory)
			continue;

		out.push_back(full_file_name);
	} while (FindNextFile(dir, &file_data));

	FindClose(dir);
	return out;
}

vector<string> sortFilenames(vector<string> filenames){
	
	//int size = (filenames.size());
	vector<int> temp; // = vector<int>(size);
	vector<string> sortedFilenames;// = vector<string>(size);
	string path = "";
	string perfix = "outputCloud";

	//temp.clear();
	vector<string>::const_iterator iter;
	for (iter = filenames.begin(); iter != filenames.end(); ++iter)
	{
		int indexSeparator = (*iter).find_last_of('\\');
		path = (*iter).substr(0, indexSeparator);

		string name = (*iter).substr(indexSeparator + 1, (*iter).length());

		int cloudNumber = atoi(name.substr(perfix.length(), name.length()).c_str());
		temp.push_back(cloudNumber);

	}

	sort(temp.begin(), temp.end());

	vector<int>::const_iterator iter_int;
	for (iter_int = temp.begin(); iter_int != temp.end(); ++iter_int){
		string name = path + '\\' + perfix + boost::lexical_cast<string>(*iter_int);
		sortedFilenames.push_back(name);
		//cout << name << endl;
	}

	return sortedFilenames;
}

// ----------------------------------------------------------------------
// -----POINT CLOUD SEGMENTATION-----------------------------------------
// ----------------------------------------------------------------------
/*pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
std::vector<pcl::PointIndices> EuclideanClusterExtractionSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ){

	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloudXYZ);

	std::vector<pcl::PointIndices> cluster_indices;
	ec.setClusterTolerance(0.028); // 2,8cm
	ec.setMinClusterSize(2000);
	//ec.setMaxClusterSize(40000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloudXYZ);
	ec.extract(cluster_indices);

	return cluster_indices;
}*/

void RegionGrowingSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr){

	//copy point cloud no format (Joao Fiadeiro study) XYZRGB to XYZ 
	pcl::PointCloud <pcl::PointXYZ>::Ptr cloudXYZ (new pcl::PointCloud <pcl::PointXYZ>);
	copyPointCloud<PointXYZRGB, PointXYZ>(*point_cloud_ptr, *cloudXYZ);

	pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> >(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
	normal_estimator.setSearchMethod(tree);
	normal_estimator.setInputCloud(cloudXYZ);
	normal_estimator.setKSearch(50);
	normal_estimator.compute(*normals);

	/*pcl::IndicesPtr indices(new std::vector <int>);
	pcl::PassThrough<pcl::PointXYZRGB> pass;
	pass.setInputCloud(point_cloud_ptr);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.0, 1.0);
	pass.filter(*indices);*/

	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
	reg.setMinClusterSize(1000);
//	reg.setMaxClusterSize(1000000);
	reg.setSearchMethod(tree);
	reg.setNumberOfNeighbours(400);
	reg.setInputCloud(cloudXYZ);
	//reg.setIndices (indices);
	reg.setInputNormals(normals);
	reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
	reg.setCurvatureThreshold(1.0);

	std::vector <pcl::PointIndices> clusters;
	reg.extract(clusters);

	std::cout << "Number of clusters is equal to " << clusters.size() << std::endl;
	std::cout << "First cluster has " << clusters[0].indices.size() << " points." << endl;
	std::cout << "These are the indices of the points of the initial" <<
		std::endl << "cloud that belong to the first cluster:" << std::endl;
	int counter = 0;
	while (counter < clusters[0].indices.size())
	{
		std::cout << clusters[0].indices[counter] << ", ";
		counter++;
		if (counter % 10 == 0)
			std::cout << std::endl;
	}
	std::cout << std::endl;

	pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	//viewer = rgbVis(point_cloud_ptr);
	pcl::visualization::CloudViewer viewer("Cluster viewer");
	viewer.showCloud(colored_cloud);
	while (!viewer.wasStopped())
	{
	//	viewer->spinOnce(100);
	//	boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

	return;

}

void colorBasedRegionGrowingSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr){

	pcl::search::Search <pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> >(new pcl::search::KdTree<pcl::PointXYZRGB>);

	pcl::IndicesPtr indices(new std::vector <int>);
	pcl::PassThrough<pcl::PointXYZRGB> pass;
	pass.setInputCloud(point_cloud_ptr);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0.0, 1.0);
	pass.filter(*indices);

	pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
	reg.setInputCloud(point_cloud_ptr);
	reg.setIndices(indices);
	reg.setSearchMethod(tree);
	reg.setDistanceThreshold(6);
	reg.setPointColorThreshold(6);
	reg.setRegionColorThreshold(6);
	reg.setMinClusterSize(1000);

	std::vector <pcl::PointIndices> clusters;
	reg.extract(clusters);

	pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
	pcl::visualization::CloudViewer viewer("Cluster viewer");
	viewer.showCloud(colored_cloud);
	while (!viewer.wasStopped())
	{
		boost::this_thread::sleep(boost::posix_time::microseconds(100));
	}

}

// --------------
// -----Debug-----
// --------------


void visualizeOuputCloudX(list<OutputCloud*> outputCloudList, int elemIdx) {


	if (outputCloudList.size() > elemIdx) {

		list<OutputCloud*>::const_iterator iterator = outputCloudList.begin();
		advance(iterator, elemIdx);

		visualizePointCloudClusters(*iterator, elemIdx);
	}

	return;
}

void visualizeClusterX(PointCloud<PointXYZ>::Ptr cluster, int clusterID) {

	visualizePointCloudCluster(cluster, clusterID);
	return;
}

void readOutputCloudClustersFiles(){

	string path = "C:\\Users\\Public\\Data\\JoaoFiadeiro\\SecondSession\\SecondTestPointClouds\\test";

	vector<string> clusterFiles = getFilesInDirectory(path);

	int numberOfClusterFiles = clusterFiles.size();
	for (int i = 0; i < numberOfClusterFiles; i++){
		
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		if (pcl::io::loadPCDFile<pcl::PointXYZ>(clusterFiles[i], *cloud) == -1) //* load the file 
		{
			cout << "Couldn't read file " << clusterFiles[i] << endl;
			return; 
		}
		else {
			cout << "file " << clusterFiles[i] << endl;
			visualizePointCloudCluster(cloud, i);
		}

	}
}

/*
void WriteClusters2File(list<OutputCloud> outputCloudList, string filepath) {

	for (list<OutputCloud>::const_iterator iterator = outputCloudList.begin(); iterator != outputCloudList.end(); ++iterator) {

		OutputCloud cloud = *iterator;
		cloud.writeClusters2File(filepath);

	}

	return;
}*/

// ----------------------------------------------------------------------
// -----Methods to use with Node, OutputCloud and CloudCluster classes-----
// ----------------------------------------------------------------------

list<CloudCluster*> getPreviousClusters(OutputCloud* outputCloud, list<Node*> nodeList, FILE* logFile) {

	int diff = 10;
	int outputCloudIdxTemp = -1;
	list<CloudCluster*> tempList;

	/*if (!nodeList.empty()) {
		OutputCloud* previousOutputCloud = nodeList.back()->getOutputCloud();
		outputCloudIdxTemp = previousOutputCloud->getIndex();
		tempList = previousOutputCloud->getClusters();
	}*/


	for (list<Node*>::const_reverse_iterator iter = nodeList.rbegin(); iter != nodeList.rend(); ++iter) {

		outputCloudIdxTemp = (*iter)->getOutputCloud()->getIndex();
		int clustersSize = outputCloud->getClusters().size();
		tempList = (*iter)->getOutputCloud()->getClusters();
		int previousClustersSize = (*iter)->getOutputCloud()->getClusters().size();
		if (clustersSize == previousClustersSize) {
			// delete outputCloudIdxTemp - just for debug
			outputCloudIdxTemp = (*iter)->getOutputCloud()->getIndex();
			fprintf(logFile, "outputCloud i :%d - clusters size :%d | outputCloud i:%d - previous clusters size:%d \n", outputCloud->getIndex(),
				clustersSize, outputCloudIdxTemp, previousClustersSize);
			return (*iter)->getOutputCloud()->getClusters();

		}
		else {
			int tempDiff = abs(clustersSize - previousClustersSize);
			if (diff > tempDiff) {
				diff = tempDiff;
				tempList = (*iter)->getOutputCloud()->getClusters();
				// delete outputCloudIdxTemp - just for debug
				outputCloudIdxTemp = (*iter)->getOutputCloud()->getIndex();
			}
		}
	}

	fprintf(logFile, "outputCloud i :%d - clusters size :%d | outputCloud i:%d - previous clusters size:%d \n", outputCloud->getIndex(), 
		outputCloud->getClusters().size(), outputCloudIdxTemp, tempList.size()); 
	
	return tempList;
}

// delete outputCloud elements so the computer doesn't run out of memory
list<Node*> releaseXOutputCloudInstances(list<Node*> nodeList, int xElements) {

	int i = 0;
	list<Node*> nodeListTemp = nodeList;
	for(list<Node*>::const_iterator iter = nodeList.begin(); iter != nodeList.end(); ++iter) {
		if (i <= xElements) {
			Node* node = *iter;
			nodeListTemp.remove(node);
			delete node;
			i++;
		}
		else
			break;
	}

	return nodeListTemp;
}

void generateAndWriteOutputCloudClusters(){

	// map allocator for the point clouds filenames 
	vector<string> pointCloudFilesGirafa;
	vector<string> pointCloudFilesSilvia;
	vector<string> pointCloudFilesSurface;
	list<Node*> outputCloudNodeList;
	list<Node*> tempOutputCloudNodeList;

	// for debug purposes
	list<OutputCloud*> outputCloudList;
	FILE* logFile;
	logFile = fopen("logFile.txt", "w");
	int numMaxNodes = 50;

	// SURFACE
	// array for the directories where the point clouds are stored

/*	string outputCloudPathsAndCalibration[3][2] = {
			{ "C:\\Users\\Public\\Data\\JoaoFiadeiro\\SecondSession\\SecondTestPointClouds\\girafa\\Output2", "girafa.ini" },
			{ "C:\\Users\\Public\\Data\\JoaoFiadeiro\\SecondSession\\SecondTestPointClouds\\silvia\\Output2", "silvia.ini" },
			{ "C:\\Users\\Public\\Data\\JoaoFiadeiro\\SecondSession\\SecondTestPointClouds\\surface\\Output2", "surface.ini" }
	}; // filepath, calibration filename */

	//MINI WORK
	// array for the directories where the point clouds are stored 
	
	/*string outputCloudPathsAndCalibration[3][2] = {
	{ "E:\\Data\\JoaoFiadeiro\\SecondSession\\SecondTestPointClouds\\girafa\\Output2\\sample", "girafa.ini" },
	{ "E:\\Data\\JoaoFiadeiro\\SecondSession\\SecondTestPointClouds\\silvia\\Output2\\sample", "silvia.ini" },
	{ "E:\\Data\\JoaoFiadeiro\\SecondSession\\SecondTestPointClouds\\surface\\Output2\\sample", "surface.ini" }
	}; // filepath, calibration filename */

	//BLACKBOX
	// array for the directories where the point clouds are stored 

	string outputCloudPathsAndCalibration[3][2] = {
		{ "D:\\DATA\\Second Session\\Kinect Data\\girafa\\Output2\\sample", "girafa.ini" },
		{ "D:\\DATA\\Second Session\\Kinect Data\\silvia\\Output2\\sample", "silvia.ini" },
		{ "D:\\DATA\\Second Session\\Kinect Data\\surface\\Output2\\sample", "surface.ini" }
	}; // filepath, calibration filename 



	//load point cloud filenames into to the appropriate lists and sort them
	pointCloudFilesGirafa = sortFilenames(getFilesInDirectory(outputCloudPathsAndCalibration[0][0]));
	pointCloudFilesSilvia = sortFilenames(getFilesInDirectory(outputCloudPathsAndCalibration[1][0]));
	pointCloudFilesSurface = sortFilenames(getFilesInDirectory(outputCloudPathsAndCalibration[2][0]));



	if (pointCloudFilesGirafa.size() != pointCloudFilesSilvia.size()
		&& pointCloudFilesSilvia.size() != pointCloudFilesSurface.size()
		&& pointCloudFilesGirafa.size() != pointCloudFilesSurface.size()) {

		cout << "ERROR: number of point clouds for each viewpoint doesnt match!" << endl;
		return;
	}

	//list<CloudCluster*> previousClusters;
	int numberOfPointCloudFiles = pointCloudFilesSilvia.size();
	for (int i = 0; i < numberOfPointCloudFiles; i++){

		Node* node = new Node();
		OutputCloud* outputCloud = new OutputCloud();
		map<string, string> filenameByCalibrationpath;

		//girafa
		filenameByCalibrationpath.insert(make_pair(pointCloudFilesGirafa[i], outputCloudPathsAndCalibration[0][1]));
		//cout << "filename = " << pointCloudFilesGirafa[i] << " calibration file = " << outputCloudPathsAndCalibration[0][1] << endl;
		// silvia
		filenameByCalibrationpath.insert(make_pair(pointCloudFilesSilvia[i], outputCloudPathsAndCalibration[1][1]));
		//cout << "filename = " << pointCloudFilesSilvia[i] << " calibration file = " << outputCloudPathsAndCalibration[1][1] << endl;
		// surface
		filenameByCalibrationpath.insert(make_pair(pointCloudFilesSurface[i], outputCloudPathsAndCalibration[2][1]));
		//cout << "filename = " << pointCloudFilesSurface[i] << " calibration file = " << outputCloudPathsAndCalibration[2][1] << endl;

		outputCloud->loadPointClouds(filenameByCalibrationpath);
		filenameByCalibrationpath.clear();
		//outputCloudList*/
		
		//calculate point cloud clusters
		outputCloud->setLogFile(logFile);
		outputCloud->setIndex(i);
		outputCloud->calculatePointCloudClusters();
		outputCloud->estimateClusterNormals();
		outputCloud->determinePointCloudClustersIndex(getPreviousClusters(outputCloud, outputCloudNodeList, logFile));
		//previousClusters = outputCloud->getClusters();
		string filepath = "C:\\Users\\claudia\\Desktop\\clusters\\OutputCloud" + boost::lexical_cast<std::string>(i);
		//cout << filepath << endl;
		//outputCloud->writeClusters2PLYFile(filepath);

		fprintf(logFile, " wrote outputCloud %d clusters to file \n", i);
		cout << "wrote outputCloud " << i << " clusters to file" << endl;

		if (outputCloudNodeList.empty())
			node->setPreviousNode(NULL);
		else {
			node->setPreviousNode(tempOutputCloudNodeList.front());
			tempOutputCloudNodeList.pop_front();
		}
		node->setOutuputCloud(outputCloud);

		outputCloudList.push_back(outputCloud);
		outputCloudNodeList.push_back(node);
		tempOutputCloudNodeList.push_back(node);

		if ((outputCloudNodeList.size() / 2) == numMaxNodes) {
			outputCloudNodeList = releaseXOutputCloudInstances(outputCloudNodeList, numMaxNodes);
		}

		//debug point cloud, clusters and normals
		//visualizePointCloud(outputCloud->getPointCloudRGB());
		//outputCloud->visualizePointCloudClusters();
		//outputCloud->visualizePointCloudClustersNormals();

		//debug point cloud individual clusters
		outputCloud->visualizePointCloudCluster(1, outputCloudNodeList.size());

		//for (int i = 0; i < previousClusters.size(); i++)
		//	cout << "cluster " << i << " number of points = " << outputCloud.getClusterX(i)->points.size() << endl;
		//visualizePointCloud(outputCloud.getPointCloudRGB(), 0);

	}

	fclose(logFile);

	//visualizeOuputCloudX(outputCloudList, 10);

	return;

}

void generateClustersColorBasedRegionGrowing() {


	// map allocator for the point clouds filenames 
	vector<string> pointCloudFilesGirafa;
	vector<string> pointCloudFilesSilvia;
	vector<string> pointCloudFilesSurface;
	//list<Node*> outputCloudNodeList;
	//list<Node*> tempOutputCloudNodeList;

	// for debug purposes
	list<OutputCloud*> outputCloudList;
	FILE* logFile;
	logFile = fopen("logFile.txt", "w");

	// SURFACE
	// array for the directories where the point clouds are stored

	string outputCloudPathsAndCalibration[3][2] = {
			{ "C:\\Users\\Public\\Data\\JoaoFiadeiro\\SecondSession\\SecondTestPointClouds\\girafa\\Output2\\sample", "girafa.ini" },
			{ "C:\\Users\\Public\\Data\\JoaoFiadeiro\\SecondSession\\SecondTestPointClouds\\silvia\\Output2\\sample", "silvia.ini" },
			{ "C:\\Users\\Public\\Data\\JoaoFiadeiro\\SecondSession\\SecondTestPointClouds\\surface\\Output2\\sample", "surface.ini" }
	}; // filepath, calibration filename 

	//MINI WORK
	// array for the directories where the point clouds are stored 
	/*
	string outputCloudPathsAndCalibration[3][2] = {
	{ "D:\\Data\\JoaoFiadeiro\\SecondSession\\SecondTestPointClouds\\girafa\\Output2\\sample", "girafa.ini" },
	{ "D:\\Data\\JoaoFiadeiro\\SecondSession\\SecondTestPointClouds\\silvia\\Output2\\sample", "silvia.ini" },
	{ "D:\\Data\\JoaoFiadeiro\\SecondSession\\SecondTestPointClouds\\surface\\Output2\\sample", "surface.ini" }
	}; // filepath, calibration filename */


	//load point cloud filenames into to the appropriate lists and sort them
	pointCloudFilesGirafa = sortFilenames(getFilesInDirectory(outputCloudPathsAndCalibration[0][0]));
	pointCloudFilesSilvia = sortFilenames(getFilesInDirectory(outputCloudPathsAndCalibration[1][0]));
	pointCloudFilesSurface = sortFilenames(getFilesInDirectory(outputCloudPathsAndCalibration[2][0]));



	if (pointCloudFilesGirafa.size() != pointCloudFilesSilvia.size()
		&& pointCloudFilesSilvia.size() != pointCloudFilesSurface.size()
		&& pointCloudFilesGirafa.size() != pointCloudFilesSurface.size()) {

		cout << "ERROR: number of point clouds for each viewpoint doesnt match!" << endl;
		return;
	}

	//list<CloudCluster*> previousClusters;
	int numberOfPointCloudFiles = pointCloudFilesSilvia.size();
	for (int i = 0; i < numberOfPointCloudFiles; i++){

		Node* node = new Node();
		OutputCloud* outputCloud = new OutputCloud();
		map<string, string> filenameByCalibrationpath;

		//girafa
		filenameByCalibrationpath.insert(make_pair(pointCloudFilesGirafa[i], outputCloudPathsAndCalibration[0][1]));
		//cout << "filename = " << pointCloudFilesGirafa[i] << " calibration file = " << outputCloudPathsAndCalibration[0][1] << endl;
		// silvia
		filenameByCalibrationpath.insert(make_pair(pointCloudFilesSilvia[i], outputCloudPathsAndCalibration[1][1]));
		//cout << "filename = " << pointCloudFilesSilvia[i] << " calibration file = " << outputCloudPathsAndCalibration[1][1] << endl;
		// surface
		filenameByCalibrationpath.insert(make_pair(pointCloudFilesSurface[i], outputCloudPathsAndCalibration[2][1]));
		//cout << "filename = " << pointCloudFilesSurface[i] << " calibration file = " << outputCloudPathsAndCalibration[2][1] << endl;

		outputCloud->loadPointClouds(filenameByCalibrationpath);
		filenameByCalibrationpath.clear();

		colorBasedRegionGrowingSegmentation(outputCloud->getPointCloudRGB());

	}

}


void calculatePointCloudNormals(){
	
	string outputCloudPaths[2] = {
		"C:\\Users\\claud\\Desktop\\guitardata\\sample" };

	vector<string> pointCloudFiles = getFilesInDirectory(outputCloudPaths[0]);

	int ncloud = 198;
	int numberOfPointCloudFiles = pointCloudFiles.size();
	for (int i = 0; i < numberOfPointCloudFiles; i++){

		OutputCloud* outputCloud = new OutputCloud();

		outputCloud->loadPointClouds(pointCloudFiles[i]);

		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
		ne.setInputCloud(outputCloud->getPointCloudXYZ());

		// Create an empty kdtree representation, and pass it to the normal estimation object.
		// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
		ne.setSearchMethod(tree);

		// Output datasets
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

		// Use all neighbors in a sphere of radius of 2 times the resolutions of the point cloud
		double resolution = outputCloud->computeCloudResolution(outputCloud->getPointCloudXYZ());
		//cout << "resolution = " << resolution * 5 << endl;
		ne.setRadiusSearch(resolution * 5);

		// Compute the features
		ne.compute(*cloud_normals);
		
		string filepath = "C:\\Users\\claud\\Desktop\\guitardata\\sample\\OutputCloud" + boost::lexical_cast<std::string>(ncloud)+"_cloud_cluster_0.ply";
		
		//concatenate points, rgb and normals
		PointCloud<PointXYZRGBNormal>::Ptr pointCloudClusterRGBNormals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		pcl::concatenateFields<PointXYZRGB, Normal, PointXYZRGBNormal>(*(outputCloud->getPointCloudRGB()),
			*cloud_normals, *pointCloudClusterRGBNormals);

		// Remove NaN values from cloud_normals
		std::vector<int> indices;
		pcl::removeNaNNormalsFromPointCloud(*pointCloudClusterRGBNormals, *pointCloudClusterRGBNormals, indices);

		pcl::io::savePLYFileASCII(filepath, *pointCloudClusterRGBNormals);

		cout << "Wrote point cloud to: " << filepath << endl;
		ncloud++;

		delete outputCloud;
		tree.reset();
		cloud_normals.reset();
		pointCloudClusterRGBNormals.reset();
	}
}

// --------------
// -----Main-----
// --------------
int
main(int argc, char** argv)
{
	// main function
	//generateAndWriteOutputCloudClusters();

	// test segmentation algorithms
	//generateClustersColorBasedRegionGrowing();
	
	// calculate point cloud normals
	calculatePointCloudNormals();

	// debug clusters
	//readOutputCloudClustersFiles();

	return 0;
}
