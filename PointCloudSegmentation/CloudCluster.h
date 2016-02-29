
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

using namespace std;
using namespace pcl;

class CloudCluster {

	private:

		int cluster_index;
		PointXYZ cluster_centroid;
		PointCloud<PointXYZ>::Ptr pointCloudClusterXYZ;
		PointCloud<PointXYZRGB>::Ptr pointCloudClusterRGB;
		PointCloud<pcl::Normal>::Ptr pointCloudClusterNormals;
		PointCloud<PointXYZRGBNormal>::Ptr pointCloudClusterRGBNormals;
		// Bounding Box
		PointXYZ minPointAABB;
		PointXYZ maxPointAABB;

	public:

		// constructor(s)
		CloudCluster() :pointCloudClusterRGBNormals(new PointCloud<PointXYZRGBNormal>()){}
		~CloudCluster();

		// accessors
		PointXYZ getClusterCentroid() const { return cluster_centroid; }
		void setClusterCentroid(float x, float y, float z);

		PointCloud<PointXYZ>::Ptr getPointCloudClusterXYZ() const { return pointCloudClusterXYZ; }
		void setPointCloudClusterXYZ(PointCloud<PointXYZ>::Ptr cluster) { pointCloudClusterXYZ = cluster; }

		PointCloud<PointXYZRGB>::Ptr getPointCloudClusterRGB() const { return pointCloudClusterRGB; }
		void setPointCloudClusterRGB(PointCloud<PointXYZRGB>::Ptr cluster) { pointCloudClusterRGB = cluster; }

		PointCloud<pcl::Normal>::Ptr getPointCloudClusterNormals() const { return pointCloudClusterNormals; }
		void setPointCloudClusterNormals(PointCloud<pcl::Normal>::Ptr clusterNormals) { pointCloudClusterNormals = clusterNormals; }

		int getClusterIndex() const { return cluster_index; }
		void setClusterIndex(int index) { cluster_index = index; }

		void SetMinPointAABB(float x, float y, float z);
		PointXYZ getMinPointAABB() const { return minPointAABB; }

		void SetMaxPointAABB(float x, float y, float z);
		PointXYZ getMaxPointAABB() const { return maxPointAABB; }


		// methods
		bool isCentroidInsideAABB(float x, float y, float z);
		bool areAABBColliding(CloudCluster* currentCluster);
		void writeCluster2PCDFile(string filepath, PCDWriter writer);
		void writeCluster2PLYile(string filepath);

	

};
