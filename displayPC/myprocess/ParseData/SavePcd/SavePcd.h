#ifndef SAVEPCD_H
#define SAVEPCD_H

#include <myutility/cutility.h>
#include <myutility/rutility.h>
#include <QDebug>
#include <QEventLoop>
#include <QTimer>

class SavePcd
{
 public:
  SavePcd();
  SavePcd(std::string topicIn);
  void setPath(std::string pathIn);
  void setAff(float x, float y, float z, float roll, float pitch, float yaw);
  void setTopic(std::string topicIn);
private:
  int cnt = 0;
  std::string path;
  ros::NodeHandle nh;
  ros::Subscriber subLaserCloud;
  Eigen::Affine3f aff;
  std::string topic;
  void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg);
};
#endif // SAVEPCD_H
