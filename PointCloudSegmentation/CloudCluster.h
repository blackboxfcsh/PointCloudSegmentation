
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>

using namespace std;
using namespace pcl;

class CloudCluster {

	private:

		int cluster_index;
		PointXYZ cluster_centroid;
		PointCloud<PointXYZ>::Ptr pointCloudCluster;

	public:

		// constructor(s)
		CloudCluster() { }
		~CloudCluster();

		// accessors
		PointXYZ getClusterCentroid() const { return cluster_centroid; }
		void setClusterCentroid(float x, float y, float z);

		PointCloud<PointXYZ>::Ptr getPointCloudCluster() const { return pointCloudCluster; }
		void setPointCloudCluster(PointCloud<PointXYZ>::Ptr cluster) { pointCloudCluster = cluster; }

		int getClusterIndex() const { return cluster_index; }
		void setClusterIndex(int index) { cluster_index = index; }
		
		// methods
		void writeCluster2File(string filepath);

	

};
