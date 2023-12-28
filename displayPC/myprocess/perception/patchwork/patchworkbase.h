#ifndef PATCHWORKBASE_H
#define PATCHWORKBASE_H

#include "PatchWork.h"

// this is a class that includes whole perception process
class PatchWorkBase: public PerceptionInterface
{
public:
  PatchWorkBase();


  void getObjectCloud() override;

 private:
  // devide ground and nonground
  PatchWork patchwork;

  // some functions
  void eigen2PointCloud(Eigen::MatrixX3f mat, pcl::PointCloud<pcl::PointXYZI> & pc);
};

#endif // PATCHWORKBASE_H
