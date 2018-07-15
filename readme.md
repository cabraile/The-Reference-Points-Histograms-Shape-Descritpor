About
===============================

This repository contains the implementation of the Reference Points' Histograms Shape Descriptor (RPHSD) library, a visualization tool, implementation to test the method using the University of Washington RGB-D (UW) and the Princeton's ModelNet (MN) dataset and the obtained descriptors.

1 Method
===============================

For a given 3D point cloud, a set of reference points and a step size, the method basically:
* Computes a histogram of distances for each reference point from its distance to each cloud point;
* From each histogram, computes statistical features;
* Fill the descriptor with each feature.

The method's complexity is _O(kn)_, where _k_ is the number of reference points and _n_ is the size of the point cloud.

2 Visualization Tool
===============================

Located in the folder `VisTool`, the visualization tool enables the user to see the histograms and the reference points for each point cloud on an interactive interface (Figure 1). This tool still works only with ASCII pcd files.

![](readme-img/vistool_interface.png)
_Figure 1. VisTool interface._

This tool was developed in Python 3.6.5.

## 2.1 Components

The program interface contains the following components:
* Spinbox `Nb. Ref. Points`: the number of reference points computed if the reference points' type is "kmeans";
* Combobox `Ref. Points Type`: the type/arrangement of the reference points. _A1_, _A2_, _A3_ and _simple_ are static reference points. _kmeans_ computes reference points based on the selected number of reference points in the spinbox `Nb. Ref. Points`.
* Text field `Step Size`: the step size used to compute each histogram.
* Button `Read File`: reads a point cloud file.
* Button `Compute`: computes the histograms of the read point cloud.
* Checkbox `PCA`: reorientates the point cloud based on the PCA transform.

## 2.2 How to Use
To run the tool, simply hit `python main.py`. 

On the user interface, click `Read File` to select the cloud which is going to have its histograms computed. After that, select the desired reference points' type and, if the type is `kmeans`, insert the number of reference points. Changing the step size or checking PCA are optional steps. Finally, hit `Compute` to proceed.

## 2.3 Dependencies
 
* `numpy`;
* `scipy`; 
* `matplotlib`;
* `pyqt5`;
* `pyqtgraph`;
* `sklearn`.

3 Dataset Tests
===============================

The method was tested using the [University of Washington RGB-D Dataset](https://rgbd-dataset.cs.washington.edu/) and the [Princeton's ModelNet (10) Dataset](http://modelnet.cs.princeton.edu/).

## 3.1 Extracting Descriptors
Source-code used for extracting the descriptors is located on `Run-Dataset-Experiments`. In this folder, `MN-DS` and `UW-DS` contain the code used to extract the descriptors for the ModelNet and the University of Washington datasets, respectively. To run the method for each dataset, inside `MN-DS` or `UW-DS`, type `bash scripts/run.sh`. However, you might want to set some parameters before doing that. Open `scripts/run.sh` on your favorite text editor and change:
* `DS_PATH`: the location  where the classes are located at. For instance, `/home/user/Documents/rgbd-dataset/`. Must finish with `/`.
* `MAX_NUM_JOBS`: the number of jobs running simultaneously.
* `OUTPUT_PATH`: the location where the descriptors are going to be written. Must finish with `/`.

Still, if the user wants to skip the feature extraction step, in the folder `Computed-Descriptors` at the root of this repository, the features computed for the paper are already provided.
 
## 3.2 Classification

Code used to classify the computed descriptors are located at `Run-Dataset-Experiments/Classification/`. The classification of descriptors extracted from the UW and  MN datasets is implemented on `uw_descriptors.py` and `mn_descriptors.py`, respectively. In these files, the only parameter that is required to be changed is the path (`DATASET_PATH`) were the descriptors are extracted. The results are printed on the terminal and on log files into the folders `UW` and `MN`. The log files resulted from the original experiments are located into these folders.

4 RPHSD C++ Library
===============================

If the user is interested in running the RPHSD on other applications, we provide this library (located at `RPHSD-Library/`).

This library depends on __PCL 1.7__.

## 4.1 Methods

* `fex::histograms(cloud, ref_points, step_size=1e-2, max_distance=3.47)`: 
	Computes the list of the distance histograms from each reference point to the
cloud points. for the parameter `max_distance` default, the cloud points are
assumed to have coordinates between [-1, 1].
* `fex::features(H, func_list)`: Extracts features from the obtained histograms.
* `fex::getDescriptor(cloud, ref_points, step_size, func_list, max_distance=3.47)`: Computes the descriptor directly, no need to call `histograms()` before. 

## 4.2 Usage

To use the method, the user needs to:
* Load the point cloud (`pcl::PointCloud<pcl::PointXYZ> cloud`);
* Choose a set of reference points (`std::vector< pcl::PointCloud<pcl::PointXYZ> > ref_points`)
	* In `include/reference_points.hpp` the user can find some sets of reference points to use;
* Select a set of functions (`std::vector< fex::Descriptor (*) (const std::vector<fex::Histogram> &) > func_list`) to compute the features from histograms
	* In `include/features.hpp` the user can find examples of features to use.
* Set a step size (`double step`);

* Compute the descriptor (`fex::Descriptor`) using `fex::getDescriptor(cloud, ref_points, step, func_list)`.
	* `fex::Descriptor` is an alias of `std::vector<double>`.


```Cpp
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
        for(fex::Histogram h : H) {
                fex::debug(h);
        }
        fex::debug("> Flattened");

        // > Compute the descriptor from the histograms
        // > Notice that fex::histograms() and fex::features() could be 
        // replaced by fex::getDescriptor();
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


        // > Display the descriptor
        fex::debug("> Descriptor");
        fex::debug(descr);
        return 0;
}
```

