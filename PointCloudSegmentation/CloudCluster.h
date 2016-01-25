
#include <pcl/common/common_headers.h>

using namespace std;

class CloudCluster {

	public:

		// constructor(s)
		CloudCluster(int index, Eigen::Vector4f *centroid): cluster_index(index), cluster_centroid(*centroid) { }

		// accessors
		Eigen::Vector4f getClusterCentroid() const { return cluster_centroid; }
		//void setClusterCentroid(Eigen::Vector4f centroid) { cluster_centroid = centroid; }

		void setClusterIndex(int index) { cluster_index = index; }

		// methods
		void WriteCluster2File();

	private:

		int cluster_index;
		Eigen::Vector4f cluster_centroid;

};
