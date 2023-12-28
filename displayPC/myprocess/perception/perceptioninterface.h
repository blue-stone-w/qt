#ifndef PERCEPTIONINTERFACE_H
#define PERCEPTIONINTERFACE_H

#include <myutility/cutility.h>
#include <myutility/mymath.h>
#include <myutility/rutility.h>
#include "boundingbox.h"
#include "euclideancluster.h"
#include "objecttracker.h"

class PerceptionInterface
{
 public:
  double time;
  const pcl::PointCloud<pcl::PointXYZI>& parseOriCloud();
  const pcl::PointCloud<pcl::PointXYZI>& parseObjectCloud();
  pcl::PointCloud<pcl::PointXYZI>& editObjectCloud();
  const std::vector<ObjectBox>& parseObs();

  virtual ~PerceptionInterface() = default;
  void runPerception(const pcl::PointCloud<pcl::PointXYZI> cloudIn, double timeIn);

 private:
  pcl::PointCloud<pcl::PointXYZI> oriCloud; // store current cloud
  pcl::PointCloud<pcl::PointXYZI> cloudOut;  // output cloud

  // cluster: get cloud and index
  pcl::PointCloud<pcl::PointXYZI> objectCloud;  // nonground cloud from patch work
  std::vector<pcl::PointIndices> objectIndices; // store indices of every cluster from cluster
  EuclideanCluster euclideanCluster;

  // get object box from cluster
  std::vector<ObjectBox> obs;                //
  //std::vector<std::vector<float>> obs;     // position of center, length,width,height ...

  void getObjectBox();
  virtual void getObjectCloud() = 0;
  void initialize(const pcl::PointCloud<pcl::PointXYZI> cloudIn, double timeIn);
  void trackObject();

  ObjectTracker tracker;
};

#endif // PERCEPTIONINTERFACE_H
