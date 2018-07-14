#ifndef GUARD_SORT_HPP__
#define GUARD_SORT_HPP__

#include <algorithm>
#include "core.hpp"

namespace fex {

namespace sort {
/*-----------------------------------------------------------------------------
Returns the histogram bin that has the maximum value
-----------------------------------------------------------------------------*/
inline std::size_t getIdFromMax(const Histogram & h) {
	std::size_t max_idx = 0;
	double max_idx_value = h[0];
	for(std::size_t idx = 1; idx < h.size(); idx++) {
		if(max_idx_value < h[idx]) {
			max_idx = idx;
			max_idx_value = h[idx];
		}
	}
	return max_idx;
}

/*-----------------------------------------------------------------------------
Returns the mean position of the histogram zeros (where they weight most)
-----------------------------------------------------------------------------*/
inline double getIdFromZerosMean(const Histogram & h) {
        double count = 0.0;
	double avg  = 0.0;
	for(std::size_t i = 0; i < h.size(); i++) {
		if(h[i] == 0.0) {
			avg += i; 
			count += 1.0;
		}
	}
	avg /= count;
	return avg;
}

/*-----------------------------------------------------------------------------
Returns the expected position
-----------------------------------------------------------------------------*/
inline double getIdFromExpectation(const Histogram & h) {
	double exp_val = 0;
	for (std::size_t i = 0; i < h.size(); i++) {
		exp_val += h[i] * i;
	}
	return exp_val; 
}

/*-----------------------------------------------------------------------------
Returns the sorted histograms list H sorted using the criterion from the
`getIdFunction` function.
Parameters:
- H: the list of histograms
- getIdFunction: the function that, given a histogram, returns the id of a 
order-selection criterion

Usage:
"""
...
std::vector<fex::Histogram> H = fex::histograms(cloud, rpts, 0.1);
H = fex::sort::sort(H, &fex::sort::getIdFromMax);
...
"""
-----------------------------------------------------------------------------*/
template<typename T>
std::vector<Histogram> sort( 
	const std::vector<Histogram> & H,
	T (*getIdFunction)(const Histogram &),
	const bool & ascending = true
) {
	// > Get the values to be sorted from each histogram h	
	std::vector<T> key_ids(H.size());
	for(std::size_t key_id = 0; key_id < key_ids.size(); key_id++) {
		key_ids[key_id] = getIdFunction(H[key_id]);
	}
	
	// > Equivalent to the Numpy argsort
	// > Based on the answer from quantdev at 
	// https://stackoverflow.com/questions/25921706/creating-a-vector-of-indices-of-a-sorted-vector
	std::vector<std::size_t> H_sorted_ids(key_ids.size());
	std::size_t n = 0;
	std::generate(	std::begin(H_sorted_ids), 
			std::end(H_sorted_ids), [&]{ return n++; });
	
	double mult = (ascending) ? -1.0 : 1.0;
	
	std::sort(	std::begin(H_sorted_ids), std::end(H_sorted_ids), 
			[&](std::size_t i, std::size_t j){ 
				return (mult * key_ids[i] < mult * key_ids[j]); 
	});

	// The output receives the ordered histogram list
	std::vector<Histogram> H_sorted(H.size());
	for(std::size_t idx = 0; idx < H.size(); idx++) {
		H_sorted[idx] = H[ H_sorted_ids[idx] ];
	}

	return H_sorted;
}

}

}

#endif
