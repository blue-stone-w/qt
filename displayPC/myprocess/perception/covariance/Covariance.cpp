#include "Covariance.h"


Covariance::Covariance()
{
}
Covariance::~Covariance()
{
}

void Covariance::filteredWithNearestNeighborVariance(pcl::PointCloud<pcl::PointXYZI> cloud)
{
  pcl::PointCloud<pcl::PointXYZI> suspect_ground;
  pcl::PointCloud<pcl::PointXYZI> ground;
  pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;

  if (cloud.points.empty())
  {
    return ;
  }

  cloud_out.clear();

  // use z threshold
  for (auto& point : cloud.points)
  {
    if (point.z > zmax_threshold)
    {
      cloud_out.push_back(point);
    }
    else
    {
      suspect_ground.push_back(point);
    }
  }

  if (suspect_ground.points.empty())
  {
    return ;
  }

  kdtree.setInputCloud(suspect_ground.makeShared());
//  auto num = suspect_ground.points.size();
  for (auto& point : suspect_ground.points)
  {
    pointIdxRadiusSearch.clear();
    pointRadiusSquaredDistance.clear();

    int neighbor_size = kdtree.radiusSearch(point, neighbourRadius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
    if (!neighbor_size) // if no neighbor
    {
      continue;
    }

    double sum_z = point.z;
    double sum_zz = point.z * point.z;
    for (int i = 0; i < neighbor_size; i++)
    {
      const double z = suspect_ground.points[pointIdxRadiusSearch[i]].z;
      sum_z += z;
      sum_zz += z * z;
    }
    double mean_z = sum_z / neighbor_size;
    double cov = sum_zz / neighbor_size - mean_z * mean_z;

    // todo: better to reset covariance_threshold as covariance_threshold^2 when initialization
    if (cov > covarianceThreshold * covarianceThreshold)
    {
      cloud_out.push_back(point);
    }
    else
    {
      ground.push_back(point);
    }
  } // endfor:
  return ;
}
