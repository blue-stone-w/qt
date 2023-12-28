#include "perceptioninterface.h"
//#include <opencv2/imgproc.hpp>
void PerceptionInterface::initialize(const pcl::PointCloud<pcl::PointXYZI> cloudIn, double timeIn)
{
  time = timeIn;
  objectCloud.clear();
  objectIndices.clear();
  obs.clear();
  this->oriCloud = cloudIn;
  this->cloudOut = cloudIn;
}

const pcl::PointCloud<pcl::PointXYZI>& PerceptionInterface::parseOriCloud()
{
  return oriCloud;
}

const pcl::PointCloud<pcl::PointXYZI>& PerceptionInterface::parseObjectCloud()
{
  return objectCloud;
}
pcl::PointCloud<pcl::PointXYZI>& PerceptionInterface::editObjectCloud()
{
  return objectCloud;
}

const std::vector<ObjectBox>& PerceptionInterface::parseObs()
{
  return obs;
}

void PerceptionInterface::getObjectBox()
{
  if (objectCloud.empty())
  {
    return;
  }

//  std::cout << "object num: " << objectIndices.size() << std::endl;
  for (size_t i = 0; i < objectIndices.size(); ++i)
  {
    ObjectBox object;
    float min_x = std::numeric_limits<float>::max();
    float max_x = -std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float max_y = -std::numeric_limits<float>::max();
    float min_z = std::numeric_limits<float>::max();
    float max_z = -std::numeric_limits<float>::max();

    float closeDis = std::numeric_limits<float>::max();
    pcl::PointXYZ min_p;


    Eigen::Matrix<float, 1, 6, Eigen::RowMajor> accur = Eigen::Matrix<float, 1, 6, Eigen::RowMajor>::Zero();
    Eigen::Matrix<float, 3, 3> covariance_matrix = Eigen::Matrix<float, 3, 3>::Identity();
    unsigned int point_count = static_cast<unsigned int>(objectIndices[i].indices.size());
    pcl::PointCloud<pcl::PointXYZ>::Ptr currentCluster(new pcl::PointCloud<pcl::PointXYZ>);
    for (auto pit = objectIndices[i].indices.begin(); pit != objectIndices[i].indices.end(); ++pit)
    {
      //#pragma omp for

      currentCluster->push_back(pcl::PointXYZ(objectCloud.points[*pit].x,
                                              objectCloud.points[*pit].y,
                                              0));

      const pcl::PointXYZI& p = objectCloud.points[*pit];

      float t_closeDis = std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
      if (t_closeDis < closeDis)
      {
        closeDis = t_closeDis;
        min_p.x = p.x;
        min_p.y = p.y;
      }

//      min_x = p.x < min_x ? p.x : min_x;
      if (p.x < min_x)

      {
        min_x = p.x;
      }
      else if (p.x > max_x)
      {
        max_x = p.x;
      }

      if (p.y < min_y)
      {
        min_y = p.y;
      }
      else if (p.y > max_y)
      {
        max_y = p.y;
      }

      if (p.z < min_z)
      {
        min_z = p.z;
      }
      else if (p.z > max_z)
      {
        max_z = p.z;
      }

      accur[0] += p.x * p.x;
      accur[1] += p.x * p.y;
      accur[2] += p.x * p.z;
      accur[3] += p.y * p.y;
      accur[4] += p.y * p.z;
      accur[5] += p.z * p.z;
    } // endfor: have traversed points in this cluster

    accur /= static_cast<float>(point_count);
    covariance_matrix.coeffRef(0) = accur[0];
    covariance_matrix.coeffRef(1) = covariance_matrix.coeffRef(3) = accur[1];
    covariance_matrix.coeffRef(2) = covariance_matrix.coeffRef(6) = accur[2];
    covariance_matrix.coeffRef(4) = accur[3];
    covariance_matrix.coeffRef(5) = covariance_matrix.coeffRef(7) = accur[4];
    covariance_matrix.coeffRef(8) = accur[5];

    object.size[0] = std::abs(max_x - min_x);
    object.size[1] = std::abs(max_y - min_y);
    object.size[2] = max_z; //std::abs(max_z - min_z);
    float area = object.size[0] * object.size[1];
    if(area < 0.25 )
    {
      if(object.size[2] < 0.3)
      {
        continue;
      }
    }
    if(object.size[2] < 0.23)
    {
      continue;
    }

    object.probability = 100;


    // calculate box center point, not centroid???
    if (objectIndices[i].indices.size() > 0)
    {
      object.trans[0] = (min_x + max_x) * 0.5;
      object.trans[1] = (min_y + max_y) * 0.5;
      object.trans[2] = max_z/2;
    }

    // calculate orientation
    // PCA: has been proved that not suitable
    pcl::PCA<pcl::PointXYZ> currentClusterPca;
    currentClusterPca.setInputCloud(currentCluster);
    Eigen::Matrix3f eigenVectors = currentClusterPca.getEigenVectors(); // 矩阵的列向量为 normal 特征向量
    object.rotation = MyMath::vec2Quat(Eigen::Vector3d(0,0,atan(eigenVectors(1,0)/eigenVectors(0,0)))).cast<float>();


    std::vector<cv::Point2f> cloud2d;
    for(size_t j = 0; j < objectIndices[i].indices.size(); j++)
    {
      cloud2d.push_back(cv::Point2f(objectCloud.points[objectIndices[i].indices[j]].x,objectCloud.points[objectIndices[i].indices[j]].y));
//      cloud2d.at<float>(j,0) = objectCloud.points[objectIndices[i].indices[j]].x;
//      cloud2d.at<float>(j,1) = objectCloud.points[objectIndices[i].indices[j]].y;
    }

    cv::RotatedRect rotationcv = cv::minAreaRect(cv::Mat(cloud2d));
    object.trans[0] = rotationcv.center.x;
    object.trans[1] = rotationcv.center.y;
    object.size[0] = rotationcv.size.width;
    object.size[1] = rotationcv.size.height;
    object.rotation = MyMath::vec2Quat(Eigen::Vector3d(0,0,rotationcv.angle/180*3.1415)).cast<float>();
//    std::cout << "5" <<std::endl;


    obs.emplace_back(std::move(object));
    /* use ring index to remove clusters that contain only one ring
       std::vector<int> ringIndex(32,0);
       for(int j = 0; j < cluster_indices[i].indices.size(); j++)
       {}
     */
  } // endfor: have traversed all clouds
}

void PerceptionInterface::trackObject()
{
  tracker.track(obs,time); // add information such as veloctiy
//  obs = tracker.getObjResult(); // use updated obs which contains velocity and other info
}


void PerceptionInterface::runPerception(const pcl::PointCloud<pcl::PointXYZI> cloudIn, double timeIn)
{
  initialize(cloudIn, timeIn);

  getObjectCloud();

//  std::cout << "oricloud: " << oriCloud.size() << "; " << "objcloud" << objectCloud.size() << std::endl;

  euclideanCluster.computeEuclideanCluster(objectCloud,objectIndices);

  getObjectBox();

//  std::cout << "boxes num1: " << obs.size() << std::endl;
  trackObject();

//  std::cout << "boxes num2: " << obs.size() << std::endl;
}
