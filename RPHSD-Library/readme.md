About
===========================
C++ library implementation of the __Reference Points' Histograms Shape 
Descriptor__ 
method, by Carlos Andre Braile Przewodowski Filho.

Requirements
==========================
To use this library, PCL needs to be installed.

To Do
==========================
* Add sorting function parameter to function `getDescriptor` in `rphsd.hpp`.
* Demo for running on dataset _MN_ and _UW_
* Install library - CMake, Make?

Project Structure
=========================
* `include` contains the library headers and implementation.
	* `rphsd.hpp` implements the methods that compute the histograms list 
	from the input point cloud.
	* `reference_points.hpp` implements the default reference points used in 
	this project.
	* `features.hpp` implements the default feature extraction 
	functions used in this project.
	* `sort.hpp` implements the default histogram sorting methods
* `demo` contains the individual and complete tests on datasets using the RPHSD
library.
	* `individual.cpp` implements the tests of the basic library features.
