About
===============================

This repository contains the implementation of the Reference Points' Histograms Shape Descriptor (RPHSD) library, a visualization tool, implementation to test the method using the University of Washington RGB-D and the Princeton's ModelNet dataset and the obtained descriptors.

The Method
===============================

For a given 3D point cloud, a set of reference points and a step size, the method basically:
* Computes a histogram of distances for each reference point from its distance to each cloud point;
* From each histogram, computes statistical features;
* Fill the descriptor with each feature.

The method's complexity is _O(kn)_, where _k_ is the number of reference points and _n_ is the size of the point cloud.

The Visualization Tool
===============================



Dataset Tests
===============================


