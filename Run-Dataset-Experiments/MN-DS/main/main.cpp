// # INCLUDES
// ############################

// > STD
#include <iostream>
#include <cmath>
#include <fstream>

// > PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// > LOCAL INCLUDE
#include "../include/weka.cpp"
#include "../include/dir.hpp"
#include "../include/rphsd.hpp"

// ############################

void debug(std::string msg)
{
  std::cout << msg << std::endl;
  return ;
}

// #
// ############################

void getReferencePoints(
  std::vector<pcl::PointXYZ> &Q,
  std::size_t nb_ref_pts
)
{
  // > Basic 7
  if(nb_ref_pts == 7)
  {
    Q.push_back(pcl::PointXYZ(1,0,0));
    Q.push_back(pcl::PointXYZ(0,1,0));
    Q.push_back(pcl::PointXYZ(0,0,1));
    Q.push_back(pcl::PointXYZ(-1,0,0));
    Q.push_back(pcl::PointXYZ(0,-1,0));
    Q.push_back(pcl::PointXYZ(0,0,-1));
    Q.push_back(pcl::PointXYZ(0,0,0));
  }
  else if(nb_ref_pts == 27)
  {
    pcl::PointXYZ p;
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
    Q.push_back(pcl::PointXYZ(0,0,0));
    Q.push_back(pcl::PointXYZ(0,-1,0));
    Q.push_back(pcl::PointXYZ(0,1,0));
  }

  if(nb_ref_pts == 1)
  {
    Q.push_back(pcl::PointXYZ(-1,-1,-1));
    Q.push_back(pcl::PointXYZ(-1,-1,0));
    Q.push_back(pcl::PointXYZ(-1,-1,1));
    Q.push_back(pcl::PointXYZ(-1,0,-1));
    Q.push_back(pcl::PointXYZ(-1,0,0));
    Q.push_back(pcl::PointXYZ(-1,0,1));
    Q.push_back(pcl::PointXYZ(-1,1,-1));
    Q.push_back(pcl::PointXYZ(-1,1,0));
    Q.push_back(pcl::PointXYZ(-1,1,1));

    Q.push_back(pcl::PointXYZ(0,-1,-1));
    Q.push_back(pcl::PointXYZ(0,-1,0));
    Q.push_back(pcl::PointXYZ(0,-1,1));
    Q.push_back(pcl::PointXYZ(0,0,-1));
    Q.push_back(pcl::PointXYZ(0,0,0));
    Q.push_back(pcl::PointXYZ(0,0,1));
    Q.push_back(pcl::PointXYZ(0,1,-1));
    Q.push_back(pcl::PointXYZ(0,1,0));
    Q.push_back(pcl::PointXYZ(0,1,1));

    Q.push_back(pcl::PointXYZ(1,-1,-1));
    Q.push_back(pcl::PointXYZ(1,-1,0));
    Q.push_back(pcl::PointXYZ(1,-1,1));
    Q.push_back(pcl::PointXYZ(1,0,-1));
    Q.push_back(pcl::PointXYZ(1,0,0));
    Q.push_back(pcl::PointXYZ(1,0,1));
    Q.push_back(pcl::PointXYZ(1,1,-1));
    Q.push_back(pcl::PointXYZ(1,1,0));
    Q.push_back(pcl::PointXYZ(1,1,1));
  }
  return ;
}

std::vector<double> compute_descriptor(
  std::string file,
  double array_size,
  std::size_t nb_ref_pts,
  std::string descriptor_type
)
{
  // > Method var declaration
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<double> descriptor;
  std::vector<pcl::PointXYZ> Q;

  debug("> Reading file:" + file);

  if(pcl::io::loadPCDFile<pcl::PointXYZ>(file, *cloud) == -1)
  {
    debug("Error loading the file.");
    exit(-1);
  }

  std::cout << "> Cloud size: " << cloud->size() << std::endl;
  getReferencePoints(Q, nb_ref_pts);

  // > Descriptor computing
  debug( "> Computing descriptor...");

  descriptor = RPHSD(cloud, Q, array_size, descriptor_type);

  debug("...done!");
  return descriptor;
}

void get_dataset_files(
  std::string ds_path,
  std::string selected_class,
  std::vector<std::string> &training_files,
  std::vector<std::string> &test_files
)
{
  std::vector<std::string> poses;
  std::vector<std::string> class_files;

  // > Select the train files
  class_files =
      std::move(utils::getFiles(ds_path + selected_class + "/train/", DT_REG));
  for(std::string selected_file : class_files)
  {
    std::string complete_path = ds_path + selected_class + "/train/" +
        selected_file;
    training_files.push_back(std::move(complete_path));
  }

  // > Select the test files
  class_files = std::move(utils::getFiles(ds_path + selected_class + "/test/",
      DT_REG));
  for(std::string selected_file : class_files)
  {
    std::string complete_path = ds_path + selected_class + "/test/" +
        selected_file;
    test_files.push_back(std::move(complete_path));
  }
  return ;
}

// ############################

bool read_params(
  int argc,
  char *argv[],
  std::string & selected_class,
  std::string & dataset_path,
  std::string & output_path,
  std::size_t & array_size,
  std::size_t & nb_ref_pts,
  std::string & descriptor_type
)
{
  bool missing_params = false;
  int count = 1;
  selected_class = "";
  dataset_path = "";
  output_path="";
  array_size = 0;
  nb_ref_pts = 0;
  descriptor_type = "";

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
  return missing_params;
}

void print_progress(std::size_t index, std::size_t total)
{
  double progress = 100.0*((double)(index))/((double)(total));
  std::string msg = "> Read " + std::to_string(index) + " files from " +
      std::to_string(total) ;
  debug(msg);
  msg = "> Progress: " + std::to_string(progress) + "%";
  debug(msg);
  return ;
}

void fill_arff(
  std::string output_path,
  std::size_t array_size,
  std::size_t nb_ref_pts,
  std::string s_class,
  std::vector<std::string> files,
  std::string descriptor_type
)
{
  std::ofstream w_file;

  // > Start arff file
  w_file.open(output_path);

  // > Compute the descriptors from the objects in the dataset
  std::size_t i = 0;
  for(i = 0; i < files.size(); i++)
  {
    std::vector<double> descriptor;
    std::string file = files[i];

    // > Features computation
    descriptor = compute_descriptor(file, array_size,
        nb_ref_pts, descriptor_type);

    // > Put descriptor to ARFF
    appendDataToARFF(w_file, descriptor, s_class);

    // > Progress output
    print_progress(i+1, files.size());
  }

  w_file.close();
}

int main(int argc, char *argv[])
{
  // # SETUP
  // ##############################

  // > Main flow variables
  std::vector<std::string> train_files;
  std::vector<std::string> test_files;
  std::string weka_file_name = "";

  // > Control variables
  std::string temp_out_path = "";

  // > Parameters
  std::string selected_class = "";
  std::string dataset_path = "";
  std::string output_path="";
  std::size_t array_size = 0;
  std::size_t nb_ref_pts = 0;
  std::string descriptor_type = "";

  // > Params validation
  bool missing_params = read_params(argc, argv, selected_class, dataset_path,
    output_path, array_size, nb_ref_pts, descriptor_type);
  if(missing_params)
    return -1;

  // > Turn off PCL console messages
  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);

  // ##############################

  // # MAIN FLOW
  // ##############################
  weka_file_name = "RPHSD-" + std::to_string(array_size) + "-"+ selected_class;
  // > Get all the dataset information (classes and instances)
  get_dataset_files(dataset_path, selected_class, train_files, test_files);

  // > Start weka training file
  debug("=> " + selected_class + " training files computing");
  temp_out_path = output_path + "/train/" + weka_file_name + ".arff";
  fill_arff(temp_out_path, array_size, nb_ref_pts, selected_class,
      train_files, descriptor_type);

  debug("=> " + selected_class + " test files computing");
  temp_out_path = output_path + "/test/" + weka_file_name + ".arff";
  fill_arff(temp_out_path,  array_size, nb_ref_pts,selected_class,
      test_files,descriptor_type);

  // ##############################

  // # FLOW END
  // ##############################

  return 0;

  // ##############################
}
