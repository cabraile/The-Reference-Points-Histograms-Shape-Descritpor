#include <iostream>
#include <pcl/io/pcd_io.h>

#include "../include/rphsd.hpp"
#include "../include/reference_points.hpp"
#include "../include/features.hpp"
#include "../include/sort.hpp"

int main(int ac, char *av[]) {
	// > Init Validation
	if(ac < 2) {
		std::cout << "Demo requires file path as argument!" << std::endl;
	}
	
	// > Set to false if you don't want debug messages popping during 
	// runtime
	fex::SET_DEBUG(true);

	// > The input point cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (
		new pcl::PointCloud<pcl::PointXYZ>);
	// > The reference points cloud
	pcl::PointCloud<pcl::PointXYZ> Q;
	// > The features functions list
	std::vector< 
		fex::Descriptor (*) (const std::vector<fex::Histogram> &) 
	> func_list;

	// > Initialize the point cloud
	if (pcl::io::loadPCDFile<pcl::PointXYZ> (av[1], *cloud) == -1) {
		PCL_ERROR ("Couldn't read the input file\n");
		return (-1);
	}
	
	// > Set the reference points
	Q = fex::A1();
	
	// > Compute the histograms from the reference points
	std::vector<fex::Histogram> H = fex::histograms(*cloud, Q, 0.1);
	
	// > Sort
	H = fex::sort::sort(H, &fex::sort::getIdFromMax,false);
	std::cout << "> Histograms" << std::endl;
	for(fex::Histogram h : H) {
		fex::debug(h);
	}
	fex::debug("> Flattened");

	func_list.push_back(&fex::flattened);
	fex::Descriptor descr = fex::features(H, func_list);
	fex::debug(descr);
	
	// > Fill the list of features functions
	func_list.push_back(&fex::expectedValue);
	func_list.push_back(&fex::standardDeviation);
	func_list.push_back(&fex::entropy);
	func_list.push_back(&fex::zerosAveragePosition);
	func_list.push_back(&fex::maxIdx);
	func_list.push_back(&fex::maxValue);

	// > Compute the descriptor from the histograms
	// > Notice that fex::histograms() and fex::features() could be 
	// replaced by fex::getDescriptor();

	// > Display the descriptor
	fex::debug("> Descriptor");
	fex::debug(descr);
	return 0;
}
