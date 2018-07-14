import pandas as pd;
import numpy as np;
import matplotlib.pyplot as plt;
from mpl_toolkits.mplot3d import Axes3D;
from sklearn.decomposition import PCA;
from sklearn.model_selection import cross_val_score;
from sklearn.neighbors import KNeighborsClassifier;
from scipy.io import arff;

DATASET_PATH = "/PATH/TO/ARFF/";

# List of ARFF files
arff_list = [];
for A in ["1", "27"]:
	for SIZE in ["10", "30", "50", "70", "100"]:
		arff_list.append("INCRARR_{}_{}_complete.arff".format(A, SIZE));

# Log file with results
log_file = open("UW/uw.txt", "w");

for path in arff_list:

	print("==================================\n");
	print("Path: {}\n".format(path));
	print("==================================\n");
	log_file.write("==================================\n");
	log_file.write("Path: {}\n".format(path));
	log_file.write("==================================\n");


	# =============================================================================
	# Input
	# =============================================================================

	ds_file = open(DATASET_PATH + path, "r");
	ds,_ = arff.loadarff(ds_file);
	rows = [];
	for row in ds:
		vals = [];
		for val in row:
		    vals.append(val);
		rows.append(vals);
	ds = np.array(rows);

	X = np.array(ds[:,:-1],dtype=np.float64);
	Y = np.array(ds[:,-1]);
	print("> Finished Reading");
	
	# =============================================================================
	# Preprocess
	# =============================================================================

	for j in range(X.shape[1]):
	    col_max = np.max( np.abs( X[:,j] ) );
	    X[:,j] /= col_max;
	print("> Finished Normalization");

	# =============================================================================
	# Plot Features (PCA)
	# =============================================================================
	"""
	fig = plt.figure();
	ax = fig.add_subplot(121, projection='3d');
	X_pca = PCA(n_components=3).fit(X).transform(X);
	color_ids = np.zeros(Y.shape);
	Y_labels = np.unique(Y);
	for i in range(Y.size):
		for j in range(Y_labels.size):
			if(Y_labels[j] == Y[i]):
				color_ids[i] = j;
	ax.scatter(X_pca[:,0], X_pca[:,1], X_pca[:,2], c=color_ids);
	ax = fig.add_subplot(122);
	ax.scatter(X_pca[:,0], X_pca[:,1], c=color_ids);
	plt.savefig('{}.png'.format(path), bbox_inches='tight');
	print("> Finished PCA");
		"""

	# =============================================================================
	# Classification
	# =============================================================================
	
	cv_nb=10;
	k = int(np.log(X.shape[0]/cv_nb));
	knn = KNeighborsClassifier(n_neighbors = k);
	# From the doc, does not shuffle data.
	knn_scores = cross_val_score(knn, X, Y, cv=cv_nb);
	
	log_file.write("==> KNN\n");
	log_file.write("> Accuracy: {}({})\n".format(np.average(knn_scores), np.std(knn_scores)));
	print("==> KNN\n");
	print("Accuracy: {}({})\n".format(np.average(knn_scores), np.std(knn_scores)));
	
	dwnn = KNeighborsClassifier(n_neighbors = k, weights="distance");
	dwnn_scores = cross_val_score(dwnn, X, Y, cv=10);
	
	log_file.write("==> DWNN\n");
	log_file.write("> Accuracy: {}({})\n".format(np.average(dwnn_scores), np.std(dwnn_scores)));
	print("==> DWNN\n");
	print("Accuracy: {}({})\n".format(np.average(dwnn_scores), np.std(dwnn_scores)));
	print("> Finished");

log_file.close();
