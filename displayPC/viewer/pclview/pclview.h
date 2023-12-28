#ifndef PCLVIEW_H
#define PCLVIEW_H


// qt
#include <QDebug>
#include <QString>
#include <QWidget>

// C++
#include <string>
#include <vector>

// pcl
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>



#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

#include "QVTKWidget.h"

//#include "vtkSmartPointer.h"
//#include "vtkCubeAxesActor.h"
#include <vtkPlaneSource.h>
#include <vtkRenderWindow.h>
#include "myprocess/perception/boundingbox.h"


class PCLView
{
 public:
  PCLView();

  bool setParent(QWidget *parentIn);
  void resetViewport();
  int cloudSize();
  void filterCloud(float size);
  // obstacle is described by vector
  // void input(pcl::PointCloud<pcl::PointXYZI> cloudIn, std::vector<std::vector<float>> obsIn = std::vector<std::vector<float>>());
  // obstacle is described by defined sruct
  void inputObsCloud(pcl::PointCloud<pcl::PointXYZI> obsCloudIn = pcl::PointCloud<pcl::PointXYZI>());
  void inputOriCloud(pcl::PointCloud<pcl::PointXYZI> oriCloudIn = pcl::PointCloud<pcl::PointXYZI>());
  void inputObs(std::vector<ObjectBox> obsIn = std::vector<ObjectBox>());
  void displayCloud();
 private:
  pcl::PointCloud<pcl::PointXYZI>::Ptr oriCloud, obsCloud;
//  std::vector<std::vector<float>> obs;
  std::vector<ObjectBox> obs;
  pcl::visualization::PCLVisualizer::Ptr viewer;
  QVTKWidget *qvtkWidget;
};

#endif // PCLVIEW_H
