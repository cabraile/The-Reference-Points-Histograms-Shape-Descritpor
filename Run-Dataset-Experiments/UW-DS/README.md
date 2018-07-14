# The Reference Points Histograms Shape Descriptor
### Application on the University of Washing RGB-D Dataset

## About

This repository contains the source code of the RPHSD and the script+source to the application on the [University of Washington RGB-D Dataset](https://rgbd-dataset.cs.washington.edu/).
	
## What does it do

When you run `bash scripts/run.sh`, the descriptors from the entire dataset will be computed and they will be placed at an `arff` (attribute-relation file format) file, which can be opened by [Weka](http://www.cs.waikato.ac.nz/ml/weka/).

The output file will be placed at the directory `.arff/` (by default), which will be generated once the script start running.

## Parameters

The user parameters can be changed at `scripts/run.sh`. They are:

* - __DS\_PATH__: The root path to the `.pcd` dataset.
* - __NB\_OF\_REFERENCE\_POINTS__: The number of reference points of the descriptor. Values can be: _7_, _27_ or _1_. When this value is _1_, there will still be _27_ reference points. However, they will be arranged as a cube, while when the value is _27_ the points will be arranged as a sphere.
* - __ARRAY\_SIZE__: The size of each shape histogram.
* - __MAX\_NUM\_JOBS__: The maximum number of simultaneous jobs running to compute the descriptors.
* - __DESCRIPTOR\_TYPE__: the type of the descriptor: avg-std (computes both average and std-dev for each statistical feature obtained by the algorithm) or complete (arrays statistics without changes). 

