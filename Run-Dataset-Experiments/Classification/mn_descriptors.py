import pandas as pd;
import numpy as np;
import matplotlib.pyplot as plt;
from mpl_toolkits.mplot3d import Axes3D;
from sklearn.decomposition import PCA;
from sklearn.neighbors import KNeighborsClassifier;
from sklearn.metrics import accuracy_score;
from scipy.io import arff;

DATASET_PATH = "/PATH/TO/ARFF/";

# List of ARFF files
train_arff_list = [];
test_arff_list = [];
for A in ["1", "27"]:
	for SIZE in ["10", "30", "50", "70", "100"]:
		final_path="{}{}_{}/".format(DATASET_PATH, A, SIZE);
		train_arff_list.append("{}train-INCRARR_{}_{}_complete.arff".format(final_path, A, SIZE));
		test_arff_list.append("{}test-INCRARR_{}_{}_complete.arff".format(final_path, A, SIZE));

# Log file with results
log_file = open("MN/mn.txt", "w");

for i in range(len(train_arff_list)):
	train_file_path = train_arff_list[i];
	test_file_path = test_arff_list[i];
	print("==================================\n");
	print("Train: {}\n".format(train_file_path));
	print("Test: {}\n".format(test_file_path));
	print("==================================\n");
	log_file.write("==================================\n");
	log_file.write("Train: {}\n".format(train_file_path));
	log_file.write("Test: {}\n".format(test_file_path));
	log_file.write("==================================\n");


	# =============================================================================
	# Input
	# =============================================================================
	raw_ds={};
	train_file = open(train_file_path, "r");
	raw_ds["train"],_ = arff.loadarff(train_file);
	test_file = open(test_file_path, "r");
	raw_ds["test"],_ = arff.loadarff(test_file);
	
	# Convert to numpý array
	X = {};
	Y = {};
	for inst in ["train", "test"]:
		rows = [];
		for row in raw_ds[inst]:
			vals = [];
			for val in row:
			    vals.append(val);
			rows.append(vals);
		ds = np.array(rows);
		X[inst] = np.array(ds[:,:-1],dtype=np.float64);
		Y[inst] = np.array(ds[:,-1]);
	print("> Finished Reading");
	print("> Nb Training Instances:{}".format(X["train"].shape[0]));
	print("> Nb Test Instances:{}".format(X["test"].shape[0]));
	
	
	# =============================================================================
	# Preprocess
	# =============================================================================
	
	for j in range(X["train"].shape[1]):
		col_max = np.max( np.abs( X["train"][:,j] ) );
		col_max = np.max( np.append([col_max], np.abs(X["test"][:,j]).flatten() ) );
		X["train"][:,j] /= col_max;
		X["test"][:,j] /= col_max;

	print("> Finished Normalization");

	# =============================================================================
	# Plot Features (PCA)
	# =============================================================================
	"""
	fig = plt.figure();
	ax = fig.add_subplot(121, projection='3d');
	X_pca = {};
	name = train_file_path.split("/")[-1];
	for inst in ["train", "test"]:
		X_pca[inst] = PCA(n_components=3).fit(X["train"]).transform(X[inst]);
		color_ids = np.zeros(Y[inst].shape);
		Y_labels = np.unique(Y);
		for i in range(Y[inst].size):
			for j in range(Y_labels.size):
				if(Y_labels[j] == Y[inst][i]):
					color_ids[i] = j;
		ax.scatter(X_pca[inst][:,0], X_pca[inst][:,1], X_pca[inst][:,2], c=color_ids);
		ax = fig.add_subplot(122);
		ax.scatter(X_pca[inst][:,0], X_pca[inst][:,1], c=color_ids);
		plt.savefig('MN/{}_{}.png'.format(inst, name), bbox_inches='tight');
	print("> Finished PCA");
	"""

	# =============================================================================
	# Classification
	# =============================================================================
	
	k = int(np.log(X["train"].shape[0]));
	knn = KNeighborsClassifier(n_neighbors = k);
	knn.fit(X["train"], Y["train"]);
	Y_pred = knn.predict(X["test"]);
	acc = accuracy_score(Y["test"], Y_pred);

	log_file.write("==> KNN\n");
	log_file.write("> Accuracy: {}\n".format(acc));
	
	dwnn = KNeighborsClassifier(n_neighbors = k, weights="distance");
	dwnn.fit(X["train"], Y["train"]);
	Y_pred = dwnn.predict(X["test"]);
	acc = accuracy_score(Y["test"], Y_pred);
	
	log_file.write("==> DWNN\n");
	log_file.write("> Accuracy: {}\n".format(acc));
	print("> Finished Classifiying");

log_file.close();
