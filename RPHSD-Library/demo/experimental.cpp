/*-----------------------------------------------------------------------------
Libraries
-----------------------------------------------------------------------------*/
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/common/pca.h>

#include "../include/rphsd.hpp"
#include "../include/reference_points.hpp"
#include "../include/features.hpp"
#include "../include/sort.hpp"

/*-----------------------------------------------------------------------------
User-defined parameters
-----------------------------------------------------------------------------*/
const double STEP_SIZE = 0.0001;
const std::size_t KMEANS_NPTS = 60;
const std::string RPT_TYPE = "A1";
const std::string SORT_CRIT = "expv";
const std::string DESC_TYPE = "features";
const std::string SAVE_PATH = "";
/*
- Description
Transpose the cloud I to the mass center;
Rotate it using PCA;
Rescale so that the coordinates range [-1,1].
- Returns the transformed cloud I_trans.
*/
void preprocess(
        const pcl::PointCloud<pcl::PointXYZ> & I,
        pcl::PointCloud<pcl::PointXYZ> & I_trans,
        const bool pca_transform
) {
        size_t npts = I.size();
	std::cout << npts << std::endl;
        /*----------------------------------
        Transpose to the center of mass
        -----------------------------------*/
        pcl::PointCloud<pcl::PointXYZ> I_1;
        // > Compute mass center
        pcl::PointXYZ c; c.x = 0; c.y = 0; c.z = 0;
        for (pcl::PointXYZ p : I.points) {
                c.x += p.x; c.y += p.y; c.z += p.z;
        }
        c.x /= (1.0 * npts); c.y /= (1.0 * npts); c.z /= (1.0 * npts);
        // > Transpose the cloud to its center of mass 
        for(pcl::PointXYZ p : I.points) {
                pcl::PointXYZ r;
                r.x = p.x - c.x; r.y = p.y - c.y; r.z = p.z - c.z;
                I_1.push_back(r);
        }

        /*-----------------------------------
        PCA - Rotation Invariant Properties
        -----------------------------------*/
        pcl::PointCloud<pcl::PointXYZ> I_2;
        if ( pca_transform ) {
                pcl::PCA<pcl::PointXYZ> pca;
                pca.setInputCloud(I_1.makeShared());
                for(pcl::PointXYZ p_orig : I_1) {
                        pcl::PointXYZ p_proj;
                        pca.project(p_orig, p_proj);
                        I_2.push_back(p_proj);
                }
        }
        else {
		for(pcl::PointXYZ p : I_1) {
                	I_2.push_back(p);
		}
        }

        /*-----------------------------------
        Rescale the image
        -----------------------------------*/
        pcl::PointCloud<pcl::PointXYZ> I_3;
        double a_max = std::numeric_limits<double>::min();
        for(pcl::PointXYZ p : I_2) {
		a_max = (a_max < fabs(p.x)) ? fabs(p.x) : a_max;
		a_max = (a_max < fabs(p.y)) ? fabs(p.y) : a_max;
		a_max = (a_max < fabs(p.z)) ? fabs(p.z) : a_max;
        }
        double inv_amax = 1.0/a_max;
        for(pcl::PointXYZ q : I_2) {
                pcl::PointXYZ p = q;
                p.x *= inv_amax;
                p.y *= inv_amax;
                p.z *= inv_amax;
                I_3.push_back(p);
        }
        I_trans = I_3;
        return ;
}


int main(int ac, char *av[]) {

	// > Init Validation
	if(ac < 2) {
		std::cout << "Demo requires file path as parameter" << std::endl;
	}
	
	// > Set to false if you don't want debug messages popping during 
	// runtime
	fex::SET_DEBUG(true);

	// > The input point cloud
	pcl::PointCloud<pcl::PointXYZ> cloud;

	// > Processed point cloud
	pcl::PointCloud<pcl::PointXYZ> proc_cloud;

	// > The reference points cloud
	pcl::PointCloud<pcl::PointXYZ> Q;

	// > The features functions list
	std::vector< 
		fex::Descriptor (*) (const std::vector<fex::Histogram> &) 
	> func_list;

	// > Initialize the point cloud
	if (pcl::io::loadPCDFile<pcl::PointXYZ> (av[1], cloud) == -1) {
		PCL_ERROR ("Couldn't read the input file\n");
		return (-1);
	}
	
	// > Save the processed cloud
	preprocess(cloud, proc_cloud, false);
	pcl::io::savePCDFileASCII ("proc_cloud.pcd", proc_cloud);
	pcl::io::savePCDFileASCII ("cloud.pcd", cloud);
	std::ofstream out_file;
	out_file.open(SAVE_PATH+"proc_cloud.xyz");
	for(pcl::PointXYZ p : proc_cloud.points) {
		out_file << p.x << " " << p.y << " " << p.z << std::endl;
	}
	out_file.close();
	
	// > Set the reference points
	if(RPT_TYPE.compare("A1") == 0) {
		Q = fex::A1();
	}
	else if(RPT_TYPE.compare("A2") == 0) {
		Q = fex::A2();
	}
	else if(RPT_TYPE.compare("A3") == 0) {
		Q = fex::A3();
	}
	else if(RPT_TYPE.compare("kmeans") == 0) {
		Q = fex::KMeans(proc_cloud, KMEANS_NPTS);
	}

	// > Compute the histograms from the reference points
	std::vector<fex::Histogram> H = fex::histograms(proc_cloud, Q, STEP_SIZE);
	out_file.open(SAVE_PATH+"H.csv");
	for(std::size_t i = 0; i < H.size(); i++) {
		fex::Histogram h = H[i];
		out_file << h[0];
		for(std::size_t j = 1; j < h.size(); j++) {
			out_file << " " << h[j];
		}
		out_file << std::endl;
	}
	out_file.close();
	// > Sort
	if(SORT_CRIT.compare("max") == 0) {
		H = fex::sort::sort(H, &fex::sort::getIdFromMax,false);
	}
	else if(SORT_CRIT.compare("zeros") == 0) {
		H = fex::sort::sort(H, &fex::sort::getIdFromZerosMean,false);
	}
	else if(SORT_CRIT.compare("expv") == 0) {
		H = fex::sort::sort(H, &fex::sort::getIdFromExpectation,false);
	}
	out_file.open(SAVE_PATH+"H_sorted.csv");
	for(std::size_t i = 0; i < H.size(); i++) {
		fex::Histogram h = H[i];
		out_file << h[0];
		for(std::size_t j = 1; j < h.size(); j++) {
			out_file << " " << h[j];
		}
		out_file << std::endl;
	}
	out_file.close();

	// > Fill the list of features functions
	func_list.push_back(&fex::standardDeviation);
	func_list.push_back(&fex::entropy);
	func_list.push_back(&fex::inverseDifferenceMoment);
	func_list.push_back(&fex::weightedMax);
	func_list.push_back(&fex::expectedValue);
	func_list.push_back(&fex::inverseExpectedValue);

	fex::Descriptor descr = fex::features(H, func_list);
	
	out_file.open(SAVE_PATH+"descr.csv");
	out_file << descr[0];
	for(std::size_t j = 1; j < descr.size(); j++) {
		out_file << " " << descr[j];
	}
	out_file << std::endl;
	out_file.close();
	return 0;
}
