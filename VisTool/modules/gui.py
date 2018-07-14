from PyQt5 import QtGui;
from PyQt5.QtCore import *;
from PyQt5.QtWidgets import QLineEdit, QToolBar, QLabel, QSlider, QCheckBox;
import pyqtgraph as pg;
from pyqtgraph import ComboBox, SpinBox, FileDialog;
from pyqtgraph.widgets.MatplotlibWidget import MatplotlibWidget;

import math;
import numpy as np;
from sklearn.cluster import KMeans;
from sklearn.decomposition import PCA;
from scipy.stats import entropy;
from modules import pcd, rphsd, reference_points;

class Interface():

	def setupNonUI(self):
		self.cloud = None;
		self.H = None;
		self.Q = None;
		return ;

	def setupUI(self):
		self.app = QtGui.QApplication(["RPHSD Analyser"]);
		self.widget = QtGui.QWidget();

		# UI Elements		

		## Create a grid layout to manage the widgets size and position
		self.layout = QtGui.QGridLayout();
		self.widget.setLayout(self.layout);

		## Create some widgets to be placed inside
		lbl_cloud = QLabel("Point Cloud", self.widget);
		self.plt_cloud = MatplotlibWidget();
		lbl_curves = QLabel("Histograms", self.widget);
		self.plt_curves= MatplotlibWidget();
		self.btn_compute = QtGui.QPushButton('Compute');
		self.btn_read_file = QtGui.QPushButton('Read File');
		lbl_n_ref_pts = QLabel("Nb. Ref. Points", self.widget);
		self.spb_n_ref_pts = SpinBox(value = 0);
		lbl_ref_choice = QLabel("Ref. Points Type", self.widget);
		self.cmb_ref_choice = ComboBox(items = ["simple","A1", "A2", 
							"A3", "kmeans"]);
		lbl_step_size = QLabel("Step Size", self.widget);
		self.qle_step_size = QLineEdit(self.widget);
		self.ckb_pca = QCheckBox("PCA", self.widget);
		# UI Layout
		#  0 1 2 | 3 4 5
		#0 l . . | l . .
		#1 i i i | i i i
		#2 i i i | i i i
		#3 l l l | l l l
		#4 s t t | . . .
		#5 b b . | . . .
		
		## Column 0 Widgets
		self.layout.addWidget(lbl_cloud, 0, 0);
		self.layout.addWidget(self.plt_cloud, 1, 0, 2, 2);
		self.layout.addWidget(lbl_n_ref_pts, 3, 0);
		self.layout.addWidget(self.spb_n_ref_pts, 4, 0);
		self.layout.addWidget(self.btn_read_file, 5, 0);
		## Column 1 Widgets
		self.layout.addWidget(lbl_ref_choice, 3, 1)
		self.layout.addWidget(self.cmb_ref_choice, 4, 1);
		self.layout.addWidget(self.btn_compute, 5, 1);	
		## Column 2 Widgets
		self.layout.addWidget(lbl_step_size, 3, 2);
		self.layout.addWidget(self.qle_step_size, 4, 2);
		self.layout.addWidget(self.ckb_pca, 5, 2);
		## Column 3 Widgets
		self.layout.addWidget(lbl_curves, 0, 2);
		self.layout.addWidget(self.plt_curves, 1, 2, 2, 3);
			
		# UI Defaults
		self.btn_compute.setEnabled(False);
		self.btn_compute.clicked.connect(self.triggerCompute);
		self.btn_read_file.clicked.connect(self.triggerReadFile);
		self.spb_n_ref_pts.setMinimum(0);
		self.spb_n_ref_pts.setMaximum(0);
		self.spb_n_ref_pts.setSingleStep(1);
		self.qle_step_size.setText("0.05");
		self.ckb_pca.clicked.connect(self.update);
		return ;

	def __init__(self):
		self.setupNonUI();
		self.setupUI();
		return ;

	def update(self):	
		## Figures
		fig_cloud = self.plt_cloud.getFigure();
		fig_cloud.clear();
		fig_curves= self.plt_curves.getFigure();
		fig_curves.clear();
		fig_curves.set_tight_layout(True);
		cloud_subplot = self.plt_cloud.getFigure().add_subplot(111, projection="3d");
		cloud_subplot.set_ylim(-1,1); cloud_subplot.set_xlim(-1,1); cloud_subplot.set_zlim(-1,1);

		if(self.Q is not None):
			# INDEX
			for index in range(self.H.shape[0]):
				cloud_subplot.scatter(self.Q[index,0], self.Q[index,1], self.Q[index,2], alpha=1.0);
		else:
			self.H = None;
		if(self.cloud is not None):
			cloud_subplot.scatter(self.cloud[:,0], 
				self.cloud[:,1], self.cloud[:,2], 
				c="black", alpha=0.01);
			if(self.ckb_pca.isChecked()):
				pca_cloud = PCA(n_components=3).\
					fit(self.cloud).transform(self.cloud);
				cloud_subplot.scatter(pca_cloud[:,0],
				pca_cloud[:,1], pca_cloud[:,2],
				c="red", alpha=0.1);
				
		else:
			self.Q = None;
			self.H = None;
		if(self.H is not None):
			hist_subplot = self.plt_curves.getFigure().add_subplot(1,1,1);
			lgd_labels=[];
			for index in range(self.H.shape[0]):
				hist_subplot.plot(self.H[index]);
				lgd_labels.append("H{}".format(index));
			hist_subplot.legend(lgd_labels);
			
		self.plt_cloud.draw();
		self.plt_curves.draw();
		return ;
	
	## Set events
	def triggerCompute(self):
		# > Reference Points
		if(self.cmb_ref_choice.value() == "kmeans"):
			npts = int(self.spb_n_ref_pts.value());
			kmeans = KMeans(n_clusters=npts, init='k-means++');
			kmeans.fit_predict(self.cloud);
			self.Q = kmeans.cluster_centers_;
		elif(self.cmb_ref_choice.value() == "A1"):
			self.Q = np.array(reference_points.A1);
		elif(self.cmb_ref_choice.value() == "A2"):
			self.Q = np.array(reference_points.A2);
		elif(self.cmb_ref_choice.value() == "A3"):
			self.Q = np.array(reference_points.A3);
		elif(self.cmb_ref_choice.value() == "simple"):
			self.Q = np.array(reference_points.Simple);
		else:
			raise ValueError("Choice not available.");
		# > Histograms
		step_size = float(self.qle_step_size.text());
		self.H = np.asarray(rphsd.compute_histograms(self.cloud, self.Q, step_size));	
		
		self.update();
		return ;

	def triggerReadFile(self):
		# > Select File
		file_dialog = FileDialog(self.widget);
		file_path = None;
		if (file_dialog.exec()):
			selected = file_dialog.selectedFiles();
			if (len(selected) > 0):
				file_path = selected[0];
			else:
				return None;
		if(file_path is None):
			return None;
		# > Set Point Cloud for window
		self.Q = None;
		self.H = None;
		self.cloud = pcd.read_pcd(file_path)[:,0:3];
		self.cloud -= np.average(self.cloud, axis=0);
		amax = np.max(np.absolute(self.cloud));
		self.cloud = self.cloud / amax;
		self.update();

		# > Update information
		self.spb_n_ref_pts.setMaximum(self.cloud.shape[0]);
		self.spb_n_ref_pts.setMinimum(1);
		self.cmb_ref_choice.setEnabled(True);
		self.btn_compute.setEnabled(True);
		return file_path;
	
	
	## Display the widget as a new window and start the Qt event loop
	def run(self):
		self.widget.show();
		self.app.exec_();
		return ;
