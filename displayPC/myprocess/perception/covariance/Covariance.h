#ifndef COVARIANCE_H
#define COVARIANCE_H

#include "../perception/perceptioninterface.h"


class Covariance
{
 public:
  Covariance();
  ~Covariance();
  pcl::PointCloud<pcl::PointXYZI> cloud_out;
  float covarianceThreshold = 0.5;
  float zmax_threshold  = 0;
  float  neighbourRadius = 0.4;
  void filteredWithNearestNeighborVariance(pcl::PointCloud<pcl::PointXYZI> cloud);

};
#endif // COVARIANCE_H
