#include "patchworkbase.h"

PatchWorkBase::PatchWorkBase()
{

}

void PatchWorkBase::eigen2PointCloud(Eigen::MatrixX3f mat, pcl::PointCloud<pcl::PointXYZI>& pc)
{
  for(int i = 0; i < mat.rows();++i)
  {
    pcl::PointXYZI point;
    point.x = mat.row(i)(0);
    point.y = mat.row(i)(1);
    point.z = mat.row(i)(2);
    point.intensity = 999;
    pc.push_back(point);
  }
}


void PatchWorkBase::getObjectCloud()
{
  Eigen::MatrixX3f cloudM;
  cloudM.resize(parseOriCloud().size(),3);
  for (int i = 0, sizetem = parseOriCloud().size(); i < sizetem; i++)
  {
    cloudM.row(i) << parseOriCloud()[i].x, parseOriCloud()[i].y, parseOriCloud()[i].z;
  }
  patchwork.estimateGround(cloudM);
  eigen2PointCloud(patchwork.getNonground(),editObjectCloud());
}
