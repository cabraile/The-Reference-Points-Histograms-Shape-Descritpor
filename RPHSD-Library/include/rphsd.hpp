#ifndef GUARD_RPHSD
#define GUARD_RPHSD

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
//#include <pcl/common/geometry.h>

#include "core.hpp"

namespace fex {

/*------------------------------------------------------------------------------
Computes the list of the distance histograms from each reference point to the
cloud points. for the parameter 'max_distance' default, the cloud points are
assumed to have coordinates between [-1, 1].

"""
pcl::PointCloud<pcl::PointXYZ> cloud = ... ; // > Read a point cloud
pcl::PointCloud<pcl::PointXYZ> ref_points = ...; // > Load the ref points

... // > Operations

// > Compute the distance histograms
std::vector<fex::Histogram> H = fex::histograms(cloud, ref_points, step_size);
------------------------------------------------------------------------------*/
inline std::vector<Histogram> histograms(
	// > The input point cloud
	const pcl::PointCloud<pcl::PointXYZ> & cloud,
	// > Reference points set as a point cloud
	const pcl::PointCloud<pcl::PointXYZ> & ref_points,
	// > Distance size for each histogram bin
	const double & step_size=1e-2,
	// > Max distance: from (-1,-1,-1) to (1,1,1)
	const double & max_distance=3.47
) {
	// > Constants
	std::size_t h_size = std::ceil(1.0/step_size);
	double h_norm_factor = 1.0/((double)cloud.size());
	double dist_norm_factor = 1.0/max_distance;

	// > Starts the histogram list H with all cleared histograms
	std::vector<Histogram> H(ref_points.size());
	for(std::size_t idx = 0; idx < H.size(); idx++)	
		H[idx] = Histogram(h_size,0.0);
	// > For each ref. point, compute the histogram
	for(std::size_t idx_rp = 0; idx_rp < ref_points.size(); idx_rp++) {
		pcl::PointXYZ q = ref_points[idx_rp];
		// > Distances for each cloud point
		for(std::size_t idx_cl = 0; idx_cl < cloud.size(); idx_cl++) {
			// > Normalized distance from ref point to a point from
			// the cloud
			pcl::PointXYZ p = cloud[idx_cl];
			double dist = dist_norm_factor * std::sqrt( 
				std::pow(p.x - q.x,2) + 
				std::pow(p.y - q.y, 2) + 
				std::pow(p.z - q.z, 2)
			);
			//double dist = pcl::geometry::distance(q,p) * 
			//	dist_norm_factor;

			// > Index the distance at the histogram
			std::size_t idx_h = (dist * (h_size-1));
			H[idx_rp][idx_h] += h_norm_factor;
		}
	}
	// > Returns normalized histograms
	return H;
}

/* ----------------------------------------------------------------------------
Extracts features from the obtained histograms.
Usage:
"""
// > Create a list of feature functions, as avg, stdev, etc
std::vector< std::vector<Descriptor>(*)(const std::vector<Histogram> &) 
	> func_list;
func_list.push_back(&func_1);
...
func_list.push_back(&func_n);

// > Finally extract features
fex::Descriptor descr = fex::features(H,func_list);
...
"""
-----------------------------------------------------------------------------*/
inline Descriptor features(
	// > List of distance histograms
	const std::vector<Histogram> & H, 
	// > List of features extraction functions from the histograms
	const std::vector< 
		Descriptor
			// > The function requires a list of histograms
			(*)(const std::vector<Histogram> &)
	> & func_list
) {
	Descriptor descr;
	for(auto func : func_list) {
		Descriptor func_descr = func(H);
		for(double feature : func_descr)
			descr.push_back(feature);
	}
	return descr;
}

/*-----------------------------------------------------------------------------
Computes the descriptor directly, no need to call "histograms()" before.

Usage:
"""
pcl::PointCloud<pcl::PointXYZ> cloud = ... ; // > Read a point cloud
pcl::PointCloud<pcl::PointXYZ> ref_points = ...; // > Load the ref points

... // > Operations

// > Compute the distance histograms
std::vector<fex::Histogram> H = fex::histograms(cloud, ref_points, step_size);

// > Create a list of feature functions, as avg, stdev, etc
std::vector< std::vector<Descriptor>(*)(const std::vector<Histogram> &) 
	> func_list;
func_list.push_back(&func_1);
...
func_list.push_back(&func_n);

// > Finally extract features
fex::Descriptor descr = fex::getDescriptor(H,Q,func_list, 0.04);
------------------------------------------------------------------------------*/
inline Descriptor getDescriptor(
	// > The input point cloud
	const pcl::PointCloud<pcl::PointXYZ> & cloud,
	// > Reference points set as a point cloud
	const pcl::PointCloud<pcl::PointXYZ> & ref_points,
	// > Distance size for each histogram bin
	const double & step_size,
	// > List of features extraction functions from the histograms
	const std::vector< 
		Descriptor
			// > The function requires a list of histograms
			(*)(const std::vector<Histogram> &)
	> & func_list,
	// > Max distance: from (-1,-1,-1) to (1,1,1)
	const double & max_distance=3.47//3.465

) {
	std::vector<Histogram> H = histograms(cloud, ref_points, step_size, 
		max_distance);
	Descriptor descr = features(H, func_list);
		
	return descr;
}

}

#endif
