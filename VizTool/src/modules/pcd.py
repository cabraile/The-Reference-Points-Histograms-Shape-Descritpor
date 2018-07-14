"""
Python module to work with point clouds.
Author: Carlos André Braile Przewodowski Filho
"""
import numpy;
import matplotlib.pyplot;
from mpl_toolkits.mplot3d import Axes3D;

"""
Reads a .xyz file.
The result is the read cloud, a numpy array of points (which are numpy arrays
too).
"""
def read_xyz(file_name):
    list_points = [];
    cloud_file = open(file_name, "r");
    for line in cloud_file:
        aux = line.replace(",", ".");
        tokens = aux[:-2].split(" ");
        list_points.append( list( map(float, tokens) ) );
    cloud = numpy.array(list_points);
    cloud_file.close();
    return cloud;


"""
Reads a .pcd file formatted according to the Point Cloud Library.
The result is the read cloud, a numpy array of points (which are numpy arrays
too).
"""
def read_pcd(file_name):
    list_points = [];
    cloud_file = open(file_name, "r");
    for line in cloud_file:
        if(line[0].isdigit() or line[0] == '-'):
            tokens = line.split(" ");
            list_points.append( list( map(float, tokens) ) );
    cloud = numpy.array(list_points);
    cloud_file.close();
    return cloud;

def save_xyz(cloud, file_path):
    out_file = open(file_path,"w");
    for point in cloud:
        msg = str(point[0])+" "+str(point[1])+" "+str(point[2])+"\n";
        out_file.write(msg);
    out_file.close();
    return ;


"""
Reduce the dimensionality from PCD files to 3: X, Y, Z.
Returns the reduced cloud.
"""
def reduce_to_xyz(in_cloud):
    cloud = numpy.delete(in_cloud, numpy.s_[3::], axis=1);
    return cloud;

"""
Removes invalid points, such as those with inf or nan.
Returns the valid cloud.
"""
def remove_invalid(in_cloud):
    temp = [];
    for point in in_cloud:
        if not (numpy.isnan(point).any() or
                numpy.isinf(point).any()):
            temp.append(point);
    cloud = numpy.array(temp);
    return cloud;


"""
Plots a 3D point cloud.
"""
def plot(cloud):
    x,y,z = zip(*cloud);
    x = numpy.array(x);
    y = numpy.array(y);
    z = numpy.array(z);

    fig = matplotlib.pyplot.figure();

    ax = fig.add_subplot(111, projection='3d');
    ax.set_aspect("equal");
    ax.scatter(x, y, -z, zdir='z', c= 'red');
    matplotlib.pyplot.show();
    return ;
