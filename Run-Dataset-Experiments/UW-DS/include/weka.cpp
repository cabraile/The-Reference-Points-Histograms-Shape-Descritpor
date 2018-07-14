#ifndef __GUARD_WEKA_GENERATOR__
#define __GUARD_WEKA_GENERATOR__

#include <iostream>
#include <fstream>
#include <vector>

void generateWekaHeader(
  std::ofstream &w_file,
  std::string name,
  std::size_t attr_size,
  std::vector<std::string> classes,
  std::string attr_prefix = "a_"
)
{
  w_file.open(name + ".arff");
  w_file << "@RELATION objects" << std::endl;
  for(std::size_t i = 0; i < attr_size; i++)
    w_file << "@ATTRIBUTE " << attr_prefix << i << " NUMERIC" << std::endl;
  w_file << "@ATTRIBUTE class {" << classes[0];
  for(std::size_t i = 1; i < classes.size(); i++)
  {
    std::string d_class = classes[i];
    w_file << "," << d_class;
  }
  w_file << "}" << std::endl;
  w_file << "@DATA" << std::endl;
  return ;
}

template<typename FeatureT, typename LabelT>
void appendDataToARFF(
  std::ofstream &w_file,
  std::vector<FeatureT> features,
  LabelT label
)
{
  for(double feature : features)
  {
    w_file << feature << ",";
  }
  w_file << label << std::endl;
  return ;
}

void generateWholeWekaFile(std::string name,
    std::vector < std::vector<double> > descriptors,
    std::vector < std::string > classes,
    std::vector < std::string > dataset_classes,
    std::string attr_prefix = "a_")
{
  std::ofstream w_file;
  std::size_t nb_desc = descriptors.size();
  if(nb_desc > 0)
  {
    std::size_t desc_size = descriptors[0].size();
    w_file.open(name + ".arff");
    w_file << "@RELATION objects" << std::endl;
    for(std::size_t i = 0; i < desc_size; i++)
      w_file << "@ATTRIBUTE " << attr_prefix << i << " NUMERIC" << std::endl;
    w_file << "@ATTRIBUTE class {" << dataset_classes[0];
    for(std::size_t i = 1; i < dataset_classes.size(); i++)
    {
      std::string dataset_class = dataset_classes[i];
      w_file << "," << dataset_class;
    }
    w_file << "}" << std::endl;
    w_file << "@DATA" << std::endl;
    int count = 0;
    for(std::vector<double> descriptor : descriptors)
    {
      std::string class_name = classes[count];
      for(double feature : descriptor)
      {
        w_file << feature << ",";
      }
      w_file << class_name << std::endl;
      count++;
    }
  }
  w_file.close();
  return ;
}

#endif
