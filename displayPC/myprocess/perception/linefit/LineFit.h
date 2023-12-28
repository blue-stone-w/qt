#ifndef LINEFIT_H
#define LINEFIT_H

#include "../perception/perceptioninterface.h"

template <class PointT>
class LineFit: public PerceptionInterface<PointT>
{
 public:
  LineFit();
  void runPerception(const pcl::PointCloud<PointT> cloudIn) override;

};
#endif // LINEFIT_H
