#ifndef GUARD_DEFAULT_RPTS__
#define GUARD_DEFAULT_RPTS__

#include <set>
#include <cmath>

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
//#include <pcl/common/geometry.h>

namespace fex {
/*-----------------------------------------------------------------------------
Static points: cross-shaped points ( 7 points )
-----------------------------------------------------------------------------*/
inline pcl::PointCloud<pcl::PointXYZ> A1() {
	pcl::PointCloud<pcl::PointXYZ> Q;
	Q.push_back(pcl::PointXYZ(1,0,0));
	Q.push_back(pcl::PointXYZ(0,1,0));
	Q.push_back(pcl::PointXYZ(0,0,1));
	Q.push_back(pcl::PointXYZ(-1,0,0));
	Q.push_back(pcl::PointXYZ(0,-1,0));
	Q.push_back(pcl::PointXYZ(0,0,-1));
	Q.push_back(pcl::PointXYZ(0,0,0));
	return Q;
}

/*-----------------------------------------------------------------------------
Static points: two poles and three equatorial lines with 45 degress between 
them( 27 points )
-----------------------------------------------------------------------------*/
inline pcl::PointCloud<pcl::PointXYZ> A2() {
	pcl::PointCloud<pcl::PointXYZ> Q;
	double delta_angle = 45.0;
	double factor = M_PI * delta_angle /180;
	for(int psi = -45; psi < 90; psi += 45) {
		for(int i = 0; i < 8; i++) {
			pcl::PointXYZ p;
			double theta = factor * (double)(i);
			p.x = std::sin(theta) * std::sin(psi);
			p.z = std::sin(theta) * std::cos(psi);
			p.y = std::cos(theta);
			Q.push_back(p);
		}
	};
	Q.push_back(pcl::PointXYZ(0,0,0));
	Q.push_back(pcl::PointXYZ(0,-1,0));
	Q.push_back(pcl::PointXYZ(0,1,0));

	return Q;
}

/*-----------------------------------------------------------------------------
Static points: cubic edges' centers step points ( 27 points )
-----------------------------------------------------------------------------*/
inline pcl::PointCloud<pcl::PointXYZ> A3() {
	pcl::PointCloud<pcl::PointXYZ> Q;
	Q.push_back(pcl::PointXYZ(-1,-1,-1));
	Q.push_back(pcl::PointXYZ(-1,-1,0));
	Q.push_back(pcl::PointXYZ(-1,-1,1));
	Q.push_back(pcl::PointXYZ(-1,0,-1));
	Q.push_back(pcl::PointXYZ(-1,0,0));
	Q.push_back(pcl::PointXYZ(-1,0,1));
	Q.push_back(pcl::PointXYZ(-1,1,-1));
	Q.push_back(pcl::PointXYZ(-1,1,0));
	Q.push_back(pcl::PointXYZ(-1,1,1));

	Q.push_back(pcl::PointXYZ(0,-1,-1));
	Q.push_back(pcl::PointXYZ(0,-1,0));
	Q.push_back(pcl::PointXYZ(0,-1,1));
	Q.push_back(pcl::PointXYZ(0,0,-1));
	Q.push_back(pcl::PointXYZ(0,0,0));
	Q.push_back(pcl::PointXYZ(0,0,1));
	Q.push_back(pcl::PointXYZ(0,1,-1));
	Q.push_back(pcl::PointXYZ(0,1,0));
	Q.push_back(pcl::PointXYZ(0,1,1));

	Q.push_back(pcl::PointXYZ(1,-1,-1));
	Q.push_back(pcl::PointXYZ(1,-1,0));
	Q.push_back(pcl::PointXYZ(1,-1,1));
	Q.push_back(pcl::PointXYZ(1,0,-1));
	Q.push_back(pcl::PointXYZ(1,0,0));
	Q.push_back(pcl::PointXYZ(1,0,1));
	Q.push_back(pcl::PointXYZ(1,1,-1));
	Q.push_back(pcl::PointXYZ(1,1,0));
	Q.push_back(pcl::PointXYZ(1,1,1));
	return Q;
}

/*-----------------------------------------------------------------------------
Dynamic points: points selected using k-Means technique
Number of points defined by `n_clusters`
Stops iteractions when reached `max_iters` or when stops converging ( when the
average of centroids movements is smaller than `thresh`.
-----------------------------------------------------------------------------*/
pcl::PointCloud<pcl::PointXYZ> KMeans(
	const pcl::PointCloud<pcl::PointXYZ> & cloud,
	const std::size_t & n_clusters,
	const double & thresh=1e-3,
	const std::size_t & max_iters=500
) {
	// > Init
	pcl::PointCloud<pcl::PointXYZ> cluster_list;
	std::set<std::size_t> idx_list;
	std::size_t c_size = cloud.size();
	std::vector<std::size_t> marker_list(c_size);

	// > Sample random points indices
	srand(time(NULL));
	while(idx_list.size() < n_clusters) {
		std::size_t idx = rand() % c_size;
		idx_list.insert(idx);
	}

	// > Use sampled indices for clusters init
	for(	std::set<std::size_t>::iterator it= idx_list.begin(); 
		it!= idx_list.end();
		++it)
		cluster_list.push_back(cloud[*it]);

	std::size_t iter = 0; 
	double div = 2*thresh+1;
	while(iter < max_iters && div > thresh) {
		// > Compute distance for each cloud point to each cluster and 
		// mark the list with the closest
		for(std::size_t p_id = 0; p_id < c_size; p_id++) {
			pcl::PointXYZ p = cloud.points[p_id];
			std::vector<double> dists(n_clusters);
			double min_dist = std::numeric_limits<double>::max();
			long int min_id = -1;
			for(std::size_t c_id = 0; c_id < n_clusters; c_id ++) {
				pcl::PointXYZ c = cluster_list[c_id];
				//double dist = pcl::geometry::distance(p,c);
				double dist = std::sqrt(
					std::pow(p.x - c.x, 2) + 
					std::pow(p.y - c.y, 2) + 
					std::pow(p.z - c.z, 2)
				);
				dists[c_id] = dist;
				if(min_dist > dist) {
					min_dist = dist;
					min_id = c_id;
				}
			}
			marker_list[p_id] = min_id;
		}

		// > For each centroid, compute the average point from the closest points
		div = 0.0;
		for(std::size_t c_id = 0; c_id <  n_clusters; c_id++) {
			pcl::PointXYZ c(0,0,0); //c.x = 0; c.y = 0; c.z = 0;
			pcl::PointXYZ last_c = cluster_list[c_id];
			std::size_t counter = 0; 
			for(std::size_t p_id = 0; p_id < c_size; p_id++) {
				if(marker_list[p_id] == c_id) {
					pcl::PointXYZ p = cloud.points[p_id];
					c.x += p.x; c.y += p.y; c.z += p.z;
					counter++;
				}
			} 
			c.x /= (1.0 * counter); c.y /= (1.0 * counter); c.z /= (1.0 *counter);
			cluster_list[c_id] = c;
			div = std::sqrt(
				std::pow(c.x - last_c.x, 2) +
				std::pow(c.y - last_c.y, 2) +
				std::pow(c.z - last_c.z, 2)
			);
			//div = pcl::geometry::distance(c,last_c);
		}
		iter++;
	}
	return cluster_list;
}

}

#endif
