// # INCLUDES
// ############################

// > STD
#include <iostream>
#include <cmath>
#include <fstream>

// > PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/common/io.h>

#include "../include/weka.cpp"
#include "../include/dir.hpp"
#include "../include/rphsd.hpp"

// ############################

// # TYPES, STRUCTS AND CLASSES
// ############################

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> CloudT;

// ############################

// # CLOUD MANIPULATION METHODS
// # ###############################

pcl::PointXYZ get_center_of_mass(
  const pcl::PointCloud <pcl::PointXYZ>::Ptr & cloud
)
{
  pcl::PointXYZ center;
  if(cloud->size() > 0)
  {
    center.x = 0.0;
    center.y = 0.0;
    center.z = 0.0;
    for(auto p : cloud->points)
    {
      center.x += p.x;
      center.y += p.y;
      center.z += p.z;
    }
    center.x /= ((double)cloud->size());
    center.y /= ((double)cloud->size());
    center.z /= ((double)cloud->size());
  }
  else
  {
    std::cout << "Warning: no points in the cloud.";
  }
  return center;
}

std::size_t removeInvalidPoints(
  pcl::PointCloud<pcl::PointXYZ>::Ptr & out_cloud,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & in_cloud
)
{
  std::size_t count = 0;
  out_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  for(pcl::PointXYZ p : in_cloud->points)
  {
    if(pcl::isFinite<pcl::PointXYZ>(p))
      out_cloud->points.push_back(p);
    else
      count++;
  }
  return count;
}

void transposeToMassCenter(
  pcl::PointCloud<pcl::PointXYZ>::Ptr & out_cloud,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud
)
{
  pcl::PointXYZ centroid;
  out_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  centroid = get_center_of_mass(cloud);
  std::cout << "> Centroid: " << centroid.x << ", " << centroid.y << ", " << centroid.x << std::endl;
  for(pcl::PointXYZ p : cloud->points)
  {
    pcl::PointXYZ q;
    q.x = p.x - centroid.x;
    q.y = p.y - centroid.y;
    q.z = p.z - centroid.z;
    out_cloud->points.push_back(q);
  }
  return ;
}

bool normalizePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &out_cloud,
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  double max_abs_val = std::numeric_limits<double>::min();
  // > Look for the maximum absolute coordinate
  for(pcl::PointXYZ p : cloud->points)
  {
    max_abs_val = (max_abs_val < std::abs(p.x) ) ? std::abs(p.x) : max_abs_val;
    max_abs_val = (max_abs_val < std::abs(p.y) ) ? std::abs(p.y) : max_abs_val;
    max_abs_val = (max_abs_val < std::abs(p.z) ) ? std::abs(p.z) : max_abs_val;
  }
  for(pcl::PointXYZ p : cloud->points)
  {
    pcl::PointXYZ q;
    q.x = p.x / max_abs_val;
    q.y = p.y / max_abs_val;
    q.z = p.z / max_abs_val;
    out_cloud->points.push_back(q);
  }

  return true;
}

void preprocessPointCloud(
  pcl::PointCloud<pcl::PointXYZ>::Ptr &out_cloud,
  const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud
)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr valid_cloud, centered_cloud;
  removeInvalidPoints(valid_cloud, cloud);
  transposeToMassCenter(centered_cloud, valid_cloud);
  normalizePointCloud(out_cloud, centered_cloud);
  return ;
}

// # ###############################

void getReferencePoints(
  const CloudT::Ptr & cloud,
  std::vector<PointT> &Q,
  std::size_t nb_ref_pts
)
{
  // > Basic 7
  if(nb_ref_pts == 7)
  {
    Q.push_back(PointT(1,0,0));
    Q.push_back(PointT(0,1,0));
    Q.push_back(PointT(0,0,1));
    Q.push_back(PointT(-1,0,0));
    Q.push_back(PointT(0,-1,0));
    Q.push_back(PointT(0,0,-1));
    Q.push_back(PointT(0,0,0));
  }
  else if(nb_ref_pts == 27)
  {
    PointT p;
    double delta_angle = 45.0;

    for(int psi = -45; psi < 90; psi += 45)
    {
      for(int i = 0; i < 8; i++)
      {
        double theta = ( M_PI * delta_angle * (double)(i) / 180.0 );
        p.x = sin(theta) * sin(psi);
        p.z = sin(theta) * cos(psi);
        p.y = cos(theta);
        Q.push_back(p);
      }
    };
    Q.push_back(PointT(0,0,0));
    Q.push_back(PointT(0,-1,0));
    Q.push_back(PointT(0,1,0));
  }
  if(nb_ref_pts == 1)
  {
    Q.push_back(PointT(-1,-1,-1));
    Q.push_back(PointT(-1,-1,0));
    Q.push_back(PointT(-1,-1,1));
    Q.push_back(PointT(-1,0,-1));
    Q.push_back(PointT(-1,0,0));
    Q.push_back(PointT(-1,0,1));
    Q.push_back(PointT(-1,1,-1));
    Q.push_back(PointT(-1,1,0));
    Q.push_back(PointT(-1,1,1));

    Q.push_back(PointT(0,-1,-1));
    Q.push_back(PointT(0,-1,0));
    Q.push_back(PointT(0,-1,1));
    Q.push_back(PointT(0,0,-1));
    Q.push_back(PointT(0,0,0));
    Q.push_back(PointT(0,0,1));
    Q.push_back(PointT(0,1,-1));
    Q.push_back(PointT(0,1,0));
    Q.push_back(PointT(0,1,1));

    Q.push_back(PointT(1,-1,-1));
    Q.push_back(PointT(1,-1,0));
    Q.push_back(PointT(1,-1,1));
    Q.push_back(PointT(1,0,-1));
    Q.push_back(PointT(1,0,0));
    Q.push_back(PointT(1,0,1));
    Q.push_back(PointT(1,1,-1));
    Q.push_back(PointT(1,1,0));
    Q.push_back(PointT(1,1,1));
  }
  return ;
}

// ############################

std::vector<double> computeDescriptor(
  std::string file,
  double array_size,
  std::size_t nb_ref_pts,
  std::string descriptor_type
)
{
  // > Method var declaration
  CloudT::Ptr in_cloud(new CloudT);
  CloudT::Ptr cloud(new CloudT);
  std::vector<double> descriptor;

  // > Input
  std::cout << "> Reading file:" << file << std::endl;
  if(pcl::io::loadPCDFile<PointT> (file, *in_cloud) == -1)
  {
    PCL_ERROR ("Couldn't read file");
    exit(-1);
  }

  // > Processing
  preprocessPointCloud(cloud, in_cloud);

  std::vector<PointT> Q;
  getReferencePoints(cloud, Q, nb_ref_pts);

  // > Descriptor computing
  descriptor = RPHSD(cloud, Q, array_size, descriptor_type);
  return descriptor;
}

void getDatasetFiles(
  std::string ds_path,
  std::string selected_class,
  std::vector<std::string> &files,
  std::vector<std::string> &instances_classes
)
{
  std::vector<std::string> poses;
  poses = std::move(utils::getFiles(ds_path + selected_class + "/", DT_DIR));
  for(std::string pose : poses)
  {
    std::vector<std::string> class_files;
    class_files = std::move(utils::getFiles(ds_path + selected_class + "/" +
        pose + "/", DT_REG));
    for(std::string selected_file : class_files)
    {
      std::string complete_path = ds_path + selected_class + "/" +
          pose + "/" + selected_file;
      files.push_back(std::move(complete_path));
      instances_classes.push_back(selected_class);
    }
  }
  return ;
}

// ############################

int main(int argc, char *argv[])
{
  // # INPUT VALIDATION
  // ##############################

  // > Default parameters
  std::string selected_class = "";
  std::string dataset_path = "";
  std::string output_path="";
  std::size_t array_size = 0;
  std::size_t nb_ref_pts = 0;
  std::string descriptor_type = "";
  bool missing_params = false;

  // > Parameters reading
  int count = 1;

  while(count < argc)
  {
    // > Argument type
    std::string param = std::string(argv[count++]);
    // > Dataset path
    if(!param.compare("-p"))
      dataset_path = std::string(argv[count++]);
    // > Select one class
    else if(!param.compare("-c"))
      selected_class = std::string(argv[count++]);
    // > Array size parameter
    else if(!param.compare("-s"))
      array_size = atoi(argv[count++]);
    else if(!param.compare("-t"))
      descriptor_type = std::string(argv[count++]);
    else if(!param.compare("-n"))
      nb_ref_pts = atoi(argv[count++]);
    // > Output ARFF Cache path
    else if(!param.compare("-o"))
      output_path = std::string(argv[count++]);
    else
    {
      std::cout << "=> Warning: argument " << param << " is not a default argument" << std::endl;
      count++;
    }
  }

  // > Parameters validation
  if(dataset_path.compare("") == 0)
  {
    std::cout << "=> Error! Argument missing: -p <dataset path>" << std::endl;
    std::cout << "> Define a path -p for the dataset in argv!" << std::endl;
    missing_params = true;
  }

  if(output_path.compare("") == 0)
  {
    std::cout << "=> Error! Argument missing: -o <output path>" << std::endl;
    std::cout << "> Define the path of the output in argv!" << std::endl;
    missing_params = true;
  }

  if(selected_class.compare("") == 0)
  {
    std::cout << "=> Error! Argument missing: -c <class name>" << std::endl;
    std::cout << "> Define the dataset class to be processed in argv!" << std::endl;
    missing_params = true;
  }

  if(descriptor_type.compare("") == 0)
  {
    std::cout << "=> Error! Argument missing: -t <descriptor type>" << std::endl;
    std::cout << "> Define type of the descriptor in argv!" << std::endl;
    std::cout << "> Possible values: 'avg-std' and 'complete'" << std::endl;
    missing_params = true;
  }

  if(nb_ref_pts == 0 || (nb_ref_pts != 1 && nb_ref_pts != 7 && nb_ref_pts != 27))
  {
    std::cout << "=> Error! Argument missing or wrong: -n <number of reference points>" << std::endl;
    std::cout << "> Define the number of reference points!" << std::endl;
    std::cout << "> Possible values: 1, 7 and 28" << std::endl;
    missing_params = true;
  }

  if(array_size == 0)
  {
    std::cout << "=> Error! Argument missing: -s <array size>" << std::endl;
    std::cout << "> Define the incremental arrays size in argv!" << std::endl;
    missing_params = true;
  }

  if(missing_params)
    return -1;

  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

  // ##############################

  // # MAIN FLOW
  // ##############################

  std::vector< std::string > instances_classes;
  std::vector< std::string > dataset_classes;
  std::vector< std::vector<double> > descriptors_list;
  std::vector<std::string> files;
  std::ofstream w_file;
  std::string weka_file_name = "RPHSD-" +
          std::to_string(array_size) + "-"+ selected_class;

  // > Get all the dataset information (classes and instances)
  getDatasetFiles(dataset_path, selected_class, files, instances_classes);

  // > Start weka file
  w_file.open(output_path + "/" + weka_file_name + ".arff");

  // > Compute the descriptors from the objects in the dataset
  for(std::size_t i = 0; i < files.size(); i++)
  {
    std::vector<double> descriptor;
    std::string file = files[i];

    // > Features computation
    descriptor = computeDescriptor(file, array_size, nb_ref_pts, descriptor_type);
    descriptors_list.push_back(descriptor);

    // > Put descriptor to ARFF
    appendDataToARFF(w_file, descriptors_list[i], instances_classes[i]);

    // > Progress output
    double progress = 100.0*((double)(i+1)) / ((double)files.size());
    //std::cout << "> Read " << i+1 << " files from " << files.size() << std::endl;
    std::cout << "\r> Progress: " << progress << "%" << std::endl;
  }

  // ##############################

  // # FLOW END
  // ##############################

  w_file.close();
  return 0;

  // ##############################
}
