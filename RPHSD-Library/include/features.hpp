#ifndef GUARD_FEATURES_HPP__
#define GUARD_FEATURES_HPP__

namespace fex {

/*-----------------------------------------------------------------------------
Returns a descriptor composed by the histogram values.
-----------------------------------------------------------------------------*/
inline std::vector<double> flattened(
	const std::vector< std::vector<double> > & H
) {
	std::vector<double> descr;
	for(std::vector<double> h : H) {
		for(double feature : h) {
			descr.push_back(feature);
		}
	}
	return descr;
}


/*-----------------------------------------------------------------------------
The descriptor has the expected value of each histogram distribution
The expected value is computed using the step size as a weight for the
distribution.
-----------------------------------------------------------------------------*/
inline std::vector<double> expectedValue(
	const std::vector< std::vector<double> > & H
) {
	std::vector<double> descr;
	
	//> Assumes that H is not empty
	//double step_size = 1.0/((double) H[0].size());
        
	for(std::vector<double> h : H) {
                double expv = 0.0;
                for(std::size_t i = 0; i < h.size(); i++) {
                        expv += h[i] * i;
                }
		//expv *= step_size; 	// > Increase variance. 0 < lim<n->inf>(expv) < inf
					// > Would be 0 < lim<n->inf>(expv) <= 1 if expv = step_size
                descr.push_back(expv);
        }
        return descr; 
}

/*-----------------------------------------------------------------------------
The descriptor has the expected value of each histogram distribution
The expected value is computed using the step size as a weight for the
distribution.
-----------------------------------------------------------------------------*/
inline std::vector<double> inverseExpectedValue(
	const std::vector< std::vector<double> > & H
) {
	std::vector<double> descr;
	
	//> Assumes that H is not empty
	//double step_size = 1.0/((double) H[0].size());
        
	for(std::vector<double> h : H) {
                double expv = 0.0;
                for(std::size_t i = 0; i < h.size(); i++) {
			expv += h[i] * (1.0 / ( ((double)i) + 1.0) );
                }
		//expv *= step_size; 	// > Increase variance. 0 < lim<n->inf>(i_expv) <= 1
					// > Would be 0 if expv *= step_size.
                descr.push_back(expv);
        }
        return descr; 
}

/*-----------------------------------------------------------------------------
Compute the weighted maximum probability.
Formula: w_{max} =\frac{ p_{max} }{ 1 + argmax{h} }
-----------------------------------------------------------------------------*/
inline std::vector<double> weightedMax(
	const std::vector< std::vector<double> > & H
) {

	std::vector<double> descr;
	double step_size = 1.0/((double) H[0].size());
	for(std::vector<double> h : H) {
		std::size_t max_idx = 0;
		double max_p = h[0];
		for(std::size_t idx = 1; idx < h.size(); idx++) {
			if(max_p < h[idx]) {
				max_p = h[idx];
				max_idx = idx;
			}
		}
		double feature = max_p * ( 1.0 / ((double) (1.0 + max_idx)));
		descr.push_back(feature);
	}
	return descr;
}

/*-----------------------------------------------------------------------------
Compute the weighted maximum probability.
Formula: w_{max} =\frac{ p_{max} }{ 1 + argmax{h} }
-----------------------------------------------------------------------------*/
inline std::vector<double> inverseDifferenceMoment(
	const std::vector< std::vector<double> > & H
) {

	std::vector<double> descr;
	for(std::vector<double> h : H) {
		double feature = 0.0;
		for(std::size_t idx = 0; idx < h.size(); idx++) {
			feature += h[idx]/( pow(1.0 + (double)idx, 2) );
		}
		descr.push_back(feature);
	}
	return descr;
}

/*-----------------------------------------------------------------------------
Computes the average position considering only the positions where h[i] = 0.
-----------------------------------------------------------------------------*/
inline std::vector<double> zerosAveragePosition(
	const std::vector< std::vector<double> > & H
) {
	std::vector<double> descr;
	
	//> Assumes that H is not empty
	double step_size = 1.0/((double) H[0].size());
        double count = 0.0;
	for(std::vector<double> h : H) {
                double avg = 0.0;
                for(std::size_t i = 0; i < h.size(); i++) {
			if(h[i] == 0.0) {
                        	avg +=  i;
				count += 1.0;
			}
                }
		avg *= step_size/count;
                descr.push_back(avg);
        }
        return descr; 
}

/*-----------------------------------------------------------------------------
The descriptor has the standard deviation value of each histogram distribution
-----------------------------------------------------------------------------*/
inline std::vector<double> standardDeviation(
	const std::vector< std::vector<double> > & H
) {
	std::vector<double> descr;
	// > Average is the same for all h in H since sum(h) = 1
	double avg = 1.0/((double)H[0].size());
	for(std::vector<double> h : H) {
                double std = 0.0;
		for(double p_i : h) {
			std += std::pow(p_i - avg, 2);
                }
                std = std::sqrt((double) std /( ((double)h.size()) - 1.0 ) );
                descr.push_back(std);
        }
        return descr; 
}

/*-----------------------------------------------------------------------------
Returns a descriptor composed by the Shannon entropy of the distributions.
-----------------------------------------------------------------------------*/
inline std::vector<double> entropy(
	const std::vector< std::vector<double> > & H
) {
	std::vector<double> descr;
	for(std::vector<double> h : H) {
		double feature = 0;
		for(double p : h) {
			if(p != 0) {
				feature -= p * std::log(p);
			}
		}
		descr.push_back(feature);
	}
	return descr;
}

/*-----------------------------------------------------------------------------
Returns a descriptor composed by the position of the maximum values of the 
distributions.
-----------------------------------------------------------------------------*/
inline std::vector<double> maxIdx(
	const std::vector< std::vector<double> > & H
) {
	std::vector<double> descr;
	double step_size = 1.0/((double) H[0].size());
	for(std::vector<double> h : H) {
		double max_idx = -1;
		double max_p = -1;
		for(double idx = 0.0; idx < h.size(); idx++) {
			if(max_p < h[idx]) {
				max_p = h[idx];
				max_idx = idx;
			}
		}
		double feature = max_idx * step_size;
		descr.push_back(feature);
	}
	return descr;
}

/*-----------------------------------------------------------------------------
Returns a descriptor composed by the maximum values of the 
distributions.
-----------------------------------------------------------------------------*/
inline std::vector<double> maxValue(
	const std::vector< std::vector<double> > & H
) {
	std::vector<double> descr;
	for(std::vector<double> h : H) {
		double max_p = -1;
		for(double p : h) {
			max_p = (max_p < p) ? p : max_p;
		}
		descr.push_back(max_p);
	}
	return descr;
}

}

#endif
