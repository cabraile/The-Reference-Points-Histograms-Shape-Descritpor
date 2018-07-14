#ifndef __GUARD_RPHSD__
#define __GUARD_RPHSD__

// > PCL
#include <pcl/point_types.h>
#include <algorithm>

#include "vector_manipulation.hpp"

// # TYPES, STRUCTS AND CLASSES
// ############################

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> CloudT;

// ############################

// # AUXILIAR METHODS
// ############################

// > Euclidean distance between two points
double d(PointT p, PointT q)
{
  double dist = pow(p.x - q.x, 2) + pow(p.y - q.y, 2) + pow(p.z - q.z, 2);

  // > The division is to normalize distance between 0 and 1,
  // where the number is the distance between the most distant points

  // > if points coordinates range from [0..1];
  //dist = sqrt(dist)/1.733;
  // > if points coordinates range from [-1..1];
  dist = sqrt(dist)/3.462;
  return dist;
}

// > Hashing function that computes the index of each
// point 'p' of the cloud in relation to a reference
// point 'q'. That index should be lower than the
// size 'M' of the hash array.
// BE AWARE: Points are expected to be normalized.
std::size_t h(PointT p, PointT q, std::size_t M)
{
  double alpha = 1.0;
  double beta = 0.0;
  double val = ((alpha * d(p, q) + beta));
  std::size_t index = M * val;
  return index;
}

typedef std::vector<double> histogram;

// > Point-flexible histograms fill-up
void fillDistHist(
  CloudT::Ptr cloud,
  std::vector< PointT > Q,
  std::vector< histogram > &H,
  std::size_t M
)
{
  // . Init
  H = std::vector<histogram>(Q.size());
  for(std::size_t i = 0; i < Q.size(); i++)
    H[i] = histogram(M, 0);

  // . incremental step
  for(PointT p : cloud->points)
  {
    for(std::size_t i = 0; i < Q.size(); i++)
    {
      std::size_t index = h(p,Q[i],M);
      H[i][index] += 1;
    }
  }

  // . normalization step
  for(std::size_t i = 0; i < Q.size(); i++)
    utils::vectorNormalization(H[i], utils::NORM_SUM_TO_1);

  return ;
}

// ############################

// # FEATURES
// ############################

double featureEntropy(std::vector<double> h)
{
  double val = 0;
  double epsilon = 0.0000001;
  for(double p : h)
    val += p * log(p + epsilon);
  val = -val;
  return val;
}

double arrayAverage(std::vector<double> h)
{
  double val = 0;
  for(double p : h)
    val += p;
  val /= ((double) h.size());
  return val;
}

double arrayVariance(std::vector<double> h, double avg)
{
  double val = 0;
  std::size_t N = h.size();
  for(std::size_t i = 0; i < N; i++)
    val += pow( h[i] - avg , 2);
  val /= ((double) N );
  return val;
}

double arrayInverseDifferenceMoment(std::vector<double> h)
{
  double val = 0;
  std::size_t N = h.size();
  for(std::size_t i = 0; i < N; i++)
    val += h[i]*(1.0/((double)(1.0+pow(i,2))));
  return val;
}

// ############################

// # DESCRIPTOR
// ############################

// > Computes the histograms individual features.
// * 'cloud' is the normalized input cloud and
// * 'M' is the size of each histogram.
// -> Returns the descriptor.
std::vector<double> RPHSD(
  CloudT::Ptr cloud,
  std::vector< PointT > Q,
  std::size_t M,
  std::string descriptor_type
)
{
  // . histograms
  std::vector<double> descriptor;
  std::vector< histogram > H;

  fillDistHist(cloud, Q, H, M);

  // . features computing
  std::vector<double> var_list(Q.size());
  std::vector<double> ent_list(Q.size());
  std::vector<double> inv_moment_list(Q.size());

  for(std::size_t i = 0; i < Q.size(); i++)
  {
    double avg = arrayAverage(H[i]);
    double var = arrayVariance(H[i], avg);
    double ent = featureEntropy(H[i]);
    double inv_moment = arrayInverseDifferenceMoment(H[i]);

    var_list[i] = var;
    ent_list[i] = ent;
    inv_moment_list[i] = inv_moment;
  }
  for(double x : var_list)
    descriptor.push_back(x);
  for(double x : ent_list)
    descriptor.push_back(x);
  for(double x : inv_moment_list)
    descriptor.push_back(x);

/*  if(descriptor_type.compare("avg-std") == 0)
  {
    descriptor.resize(6);
    double list_avg = 0;
    double list_stddev = 0;

    list_avg = arrayAverage(var_list);
    list_stddev = sqrt(arrayVariance(var_list, list_avg));
    descriptor[0] = list_avg;
    descriptor[1] = list_stddev;
    list_avg = arrayAverage(ent_list);
    list_stddev = sqrt(arrayVariance(ent_list, list_avg));
    descriptor[2] = list_avg;
    descriptor[3] = list_stddev;
    list_avg = arrayAverage(inv_moment_list);
    list_stddev = sqrt(arrayVariance(inv_moment_list, list_avg));
    descriptor[4] = list_avg;
    descriptor[5] = list_stddev;
  }

  else if(descriptor_type.compare("complete") == 0)
  {
    for(double x : var_list)
      descriptor.push_back(x);
    for(double x : ent_list)
      descriptor.push_back(x);
    for(double x : inv_moment_list)
      descriptor.push_back(x);
  }
*/
  return descriptor;
}

// ############################

#endif
