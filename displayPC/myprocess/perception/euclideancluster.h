#ifndef EUCLIDEANCLUSTER_H
#define EUCLIDEANCLUSTER_H

#include "myutility/cutility.h"
#include "myutility/rutility.h"

class EuclideanCluster
{
 public:
  EuclideanCluster();
  ~EuclideanCluster();
  EuclideanCluster(int minNum, int maxNum, double clusterTolerance);
  // get index of points and size of this cloud
  void computeEuclideanCluster(const pcl::PointCloud<pcl::PointXYZI> &cloud_in, std::vector<pcl::PointIndices> &cluster_indices);

 public:
  int minNum = 30;
  int maxNum = 20000;
  float clusterTolerance = 0.4;

 private:
  bool initCompute(const pcl::PointCloud<pcl::PointXYZI> &cloud_in);
  pcl::PointIndices indices;
  int size_pc;
};

#endif // EUCLIDEANCLUSTER_H
