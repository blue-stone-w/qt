#include "SavePcd.h"

SavePcd::SavePcd(std::string topicIn)
{
  path = std::getenv("HOME") + std::string("/temp/");
  topic = topicIn;
  subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(topic, 5, &SavePcd::cloudHandler, this);
}


void SavePcd::cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
{
  sensor_msgs::PointCloud2 tempCloud = *laserCloudMsg;
  pcl::PointCloud<pcl::PointXYZI> laserCloud, savedCloud, transformedCloud;
  pcl::moveFromROSMsg(tempCloud, laserCloud);
  if (laserCloud.empty())
  {
    return;
  }
  for(auto pt:laserCloud)
  {
    if(abs(pt.x) + abs(pt.y) > 2)
    {
      savedCloud.push_back(pt);
    }
  }
  pcl::transformPointCloud(savedCloud,transformedCloud,aff);

  long time = laserCloudMsg->header.stamp.toSec()*1000;
  std::stringstream namestream;
  namestream.precision(15);
  namestream << time << ".pcd";
  std::string name = path + namestream.str();
  pcl::io::savePCDFileBinary(name, transformedCloud);
//  std::cout << laserCloud.size() << ", " << transformedCloud.size() << std::endl;
  std::cout << ++cnt << ": " << name << std::endl;
}

void SavePcd::setPath(std::string pathIn)
{
  path = std::getenv("HOME") + pathIn;
}

void SavePcd::setAff(float x, float y, float z, float roll, float pitch, float yaw)
{
  aff = pcl::getTransformation(x, y, z, roll, pitch, yaw);
}

void SavePcd::setTopic(std::string topicIn)
{
  topic = topicIn;
}
