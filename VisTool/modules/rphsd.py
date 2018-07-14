import math;
import numpy;
import scipy.stats;

_MAX_POSSIBLE_DISTANCE_3D = 3.47;
def _idm(h):
	val = 0;
	for i in range(len(h)):
		val += h[i] /(1+i**2);
	return val;

"""
> Description: Computes the reference points histograms.
> Input: the point cloud 'P', a list of reference points 'Q' and the step
size 'S' (default S = 0.1).
> Output: the list of histograms H.
"""
def compute_histograms(P, Q, S = 0.1):
	# > Initialization
	nQ = Q.shape[0];
	M = math.ceil(1/S); # . Histograms size
	H = numpy.zeros((nQ, M));
	for p in P:
	# > Increments all bins
	#	# > Ignore invalid points
		if not (numpy.isnan(p).any() or
				numpy.isinf(p).any()):	
			for i in range(len(Q)):
				index = M * numpy.linalg.norm(Q[i]-p) / _MAX_POSSIBLE_DISTANCE_3D;
				index = int(index);
				H[i][index] += 1;

	# > Normalize histograms;
	H /= len(P);

	return H;

"""
> Description Compute the features from the histograms
> Input: the list of shape histograms 'H', the boolean value 'reduced' to
determine if the descritor is reduced or complete (default reduced=False).
> Output: the list of features D.
"""
def histograms_features(H, reduced=False):
	D = [];
	list_std = [];
	list_ent = [];
	list_idm = [];
	list_wmax = [];
	list_expv = [];
	for h in H:
		list_std.append(numpy.std(h));
		list_ent.append(scipy.stats.entropy(h));
		list_idm.append(_idm(h));
		list_wmax.append(numpy.max(h) * (1.0 /(1.0 + numpy.argmax(h))));
		expv = 0;
		for i in range(len(h)):
			expv += h[i] * (1.0/(i+1.0));
		list_expv.append(expv);

	if(reduced):
		D.append( numpy.average(numpy.array(list_std)) );
		D.append( numpy.std(numpy.array(list_std)) );
		D.append( numpy.average(numpy.array(list_ent)) );
		D.append( numpy.std(numpy.array(list_ent)) );
		D.append( numpy.average(numpy.array(list_idm)) );
		D.append( numpy.std(numpy.array(list_idm)) );
		D.append( numpy.average(numpy.array(list_wmax)) );
		D.append( numpy.std(numpy.array(list_wmax)) );
		D.append( numpy.average(numpy.array(list_expv)) );
		D.append( numpy.std(numpy.array(list_expv)) );
		
	else:
		D = list_std + list_ent + list_idm + list_wmax + list_expv;

	return D;

"""
> Encapsulates the two main methods to compute the descriptor.
> Input: the point cloud 'P', a list of reference points 'Q', the step
size 'S' (default S = 0.1) and the boolean value 'reduced' to determine if the
descritor is reduced or complete (default reduced=False).
> Output: the list of features D.
"""
def descriptor(P, Q, S = 0.1, reduced=False):
	H = compute_histograms(P,Q,S);
	D = histograms_features(H,reduced);
	return D;
