#ifndef __GUARD_VECTOR_MANIPULATION__
#define __GUARD_VECTOR_MANIPULATION__

#include<iostream>
#include<vector>

namespace utils
{
  enum normalization_type {NORM_MIN_MAX, NORM_SUM_TO_1};

  template<typename T>
  void appendVectors(std::vector<T> &v_out, std::vector<T> v1, std::vector<T> v2)
  {
    for(T elem : v1)
      v_out.push_back(elem);
    for(T elem : v2)
      v_out.push_back(elem);
    return ;
  }

  template<typename T>
  void getVectorMaxMin(T &max, T &min, std::vector<T> v)
  {
    max = std::numeric_limits<T>::min();
    min = std::numeric_limits<T>::max();
    for(T elem : v)
    {
      max = (max < elem) ? elem : max;
      min = (min > elem) ? elem : min;
    }
    return;
  }

  template<typename T>
  void vectorNormalization(
    std::vector<T> &v,
    normalization_type norm_type
  )
  {
    T sum;
    switch (norm_type)
    {
      case NORM_SUM_TO_1:
        sum = 0;
        for(T elem : v)
          sum += elem;
        for(T &elem : v)
          elem /= sum;
      break;
      case NORM_MIN_MAX:
        T max, min;
        getVectorMaxMin(max, min, v);
        for(T & elem : v)
          elem = (elem - min)/(max - min);
      break;
    }
    return ;
  }

  template <typename T>
  void euclidianDistance(T &dist, std::vector<T> v)
  {
    dist = 0;
    for(T elem : v)
      dist += pow(elem, 2);
    dist = sqrt(dist);
    return ;
  }

  /*
  *   Finds the euclidian distance sqrt( pow (v1-v2,2) )
  *   Throws std::invalid_argument exception if v1 and v2 have different sizes
  */
  template <typename T>
  void euclidianDistance(T &dist, std::vector<T> v1, std::vector<T> v2)
  {
    std::size_t size = v1.size();
    if(size != v2.size())
      throw std::invalid_argument("v1 and v2 must have the same size");

    dist = 0;
    for(std::size_t i = 0; i < size; i++)
      dist += pow(v1[i] - v2[i],2);
    dist = sqrt(dist);
    return ;
  }

  template <typename T>
  void computeAverage(double &avg, const std::vector<T> &v)
  {
    avg = 0;
    for(T val : v)
      avg += (double) val;
    avg /= (double) v.size();

    return ;
  }

  // > Avg must be calculated before this function is called
  template <typename T>
  void computeStandardDeviation(
    double &std,
    const double & avg,
    const std::vector<T> &v)
  {
    std = 0;
    for(T val : v)
      std += pow(val - avg, 2);
    std = sqrt( std / ((double) v.size()) );
    return ;
  }

  template <typename T>
  void printVector(std::vector<T> v)
  {
    std::cout << "| ";
    for(T elem : v)
      std::cout << elem << " | ";
    std::cout << std::endl;
  }

  template<typename T>
  void arrayToFile(std::vector<T> v, std::string file_name)
  {
    std::ofstream m_file;
    m_file.open(file_name);
    for(T val : v)
      m_file << val << std::endl;
    m_file.close();
    return ;
  }
}

#endif
