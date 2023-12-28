#include "pclview.h"

PCLView::PCLView()
{
  viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
  viewer->initCameraParameters();
  qvtkWidget = new QVTKWidget(NULL,Qt::Window);
  qvtkWidget->SetRenderWindow(viewer->getRenderWindow());
  viewer->setupInteractor(qvtkWidget->GetInteractor(), qvtkWidget->GetRenderWindow());
  //设置背景颜色为黑色
  viewer->setBackgroundColor(0, 0, 0);


  // 显示坐标轴:
  // todo: adjust scale by cloud range
  viewer->addCoordinateSystem(5,"global");

  obsCloud.reset(new pcl::PointCloud<pcl::PointXYZI>);

  /*
  pcl::ModelCoefficients coeffs;

  coeffs.values.push_back(0.0); // x: normal direction
  coeffs.values.push_back(0.0); // y
  coeffs.values.push_back(1.0); // z
  coeffs.values.push_back(-10.0); // offset
  viewer->addPlane(coeffs, "plane");

  coeffs.values.clear();
  coeffs.values.push_back(5.3); // cone apex: x
  coeffs.values.push_back(5.3); // y
  coeffs.values.push_back(5.0); // z
  coeffs.values.push_back(0.0); // axis direction: x
  coeffs.values.push_back(0.0); // y
  coeffs.values.push_back(-1.0); // z
  coeffs.values.push_back(45.0); // angle
  viewer->addCone(coeffs, "cone");
  */
  qvtkWidget->update();

  //加载pcd文件
  oriCloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
  for(int i = -50; i < 50; i+=3)
  {
    for(int j = -50; j < 50; j+=3)
    {
      for(int k = 0; k < 30; k+=3)
      {
        if(i*i + j*j + k*k < 1000)
        {
          pcl::PointXYZI tp;
          tp.x = i, tp.y = j, tp.z = k;
          oriCloud->push_back(tp);
        }
      }
    }
  }

  displayCloud();
}

bool PCLView::setParent(QWidget *parentIn)
{
  qvtkWidget->setParent(parentIn);

  qvtkWidget->setGeometry(0,30,parentIn->width(),parentIn->height()-30);
  return true;
}

int PCLView::cloudSize()
{
  return oriCloud->size();
}

void PCLView::displayCloud()
{
  //移除窗口点云
  viewer->removeAllPointClouds();
  viewer->removeAllShapes();

  //点云设置
  //设置点云颜色
  // todo: make this cloud colorful
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> cloud_color(oriCloud, 150, 255, 255);
  //点云颜色渲染
  viewer->addPointCloud(oriCloud, cloud_color, "cloud", 0);
  //设置点云大小
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");

  //点云颜色渲染
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> cloud_color2(obsCloud, 0, 255, 0);
  viewer->addPointCloud(obsCloud, cloud_color2, "cloud2", 0);
  //设置点云大小
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud2");

  // obstacle: max,min, rgb, id,
  /*
  struct point
  {
    float x,y,z;
  }pointc, pointSize;
  for(int i = 0, sizetem = obs.size(); i < sizetem; i++)
  {
    pointc.x = obs[i][0];
    pointc.y = obs[i][1];
    pointc.z = obs[i][2];

    pointSize.x = obs[i][3]*0.5;
    pointSize.y = obs[i][4]*0.5;
    pointSize.z = obs[i][5]*0.5;

    viewer->addCube(pointc.x-pointSize.x,pointc.x+pointSize.x,
                    pointc.y-pointSize.y,pointc.y+pointSize.y,
                    pointc.z-pointSize.z,pointc.z+pointSize.z,
                    0.8,0.1,0.1, "cube", 0);
  }
  */

  for(int i = 0, sizetem = obs.size(); i < sizetem; i++)
  {
    std::string cubeid = (QString("cube")+QString::number(i,10)).toStdString();
    viewer->addCube(obs[i].trans,  obs[i].rotation,  obs[i].size[0],obs[i].size[1],obs[i].size[2], cubeid);
  }


  int gridSize = 40;
  for(int i = -gridSize ; i <= gridSize; i += 5)
  {
    viewer->addLine(pcl::PointXYZ(i, -gridSize, 0), pcl::PointXYZ(i, gridSize, 0), QString("GriglineY%1").arg(i).toStdString(), 0);
    viewer->addLine(pcl::PointXYZ(-gridSize, i, 0), pcl::PointXYZ(gridSize, i, 0), QString("GriglineX%1").arg(i).toStdString(), 0);
  }


//  viewer->addArrow(pcl::PointXYZ(0,0,0), pcl::PointXYZ(0,0,5), 1,1,1, 1,1,1, "z");

  /*
  //set visual angle
  viewer->resetCamera(); // this function will adjust visual angle according this cloud

  pcl::visualization::Camera camera;
  viewer->getCameraParameters(camera);
  printf("%lf,%lf,%lf,", camera.pos[0], camera.pos[1], camera.pos[2]);
  printf("%lf,%lf,%lf\n", camera.view[0], camera.view[1], camera.view[2]);

  viewer->setCameraPosition(-29.570503, -52.226951, 51.029257, 0.540905, 0.478015, 0.692043);
  */

  /* set colorful cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  for(int i = 0; i < 50; i+=3)
  {
    for(int j = 0; j < 50; j+=3)
    {
      for(int k = 0; k < 30; k+=3)
      {
        if(i*i + j*j + k*k < 1000)
        {
          pcl::PointXYZRGB tp;
          tp.x = i, tp.y = j, tp.z = k;
          tp.r = i*5, tp.g = j*5, tp.b = k*5;
          cloud->push_back(tp);
        }
      }
    }
  }
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "sample cloud"); // 设置点云大小
  */

  /*
  vtkSmartPointer<vtkRenderer> pRender  = vtkSmartPointer<vtkRenderer>::New();

  vtkSmartPointer<vtkCubeAxesActor> cubeAxesActor = vtkSmartPointer<vtkCubeAxesActor>::New();
  cubeAxesActor->SetUseAxisOrigin(0);

  vtkSmartPointer<vtkCamera> camera = pRender->GetActiveCamera();
  cubeAxesActor->SetCamera(camera);
  */


  qvtkWidget->update();
}

void PCLView::filterCloud(float size)
{
  pcl::VoxelGrid<pcl::PointXYZI> sor;
  sor.setInputCloud(oriCloud);
  sor.setLeafSize(size, size, size);
  sor.filter(*oriCloud);
  displayCloud();
}


void PCLView::inputObsCloud(pcl::PointCloud<pcl::PointXYZI> obsCloudIn)
{
  *obsCloud = obsCloudIn;
}

void PCLView::inputOriCloud(pcl::PointCloud<pcl::PointXYZI> oriCloudIn)
{
  *oriCloud = oriCloudIn;
}

void PCLView::inputObs(std::vector<ObjectBox> obsIn)
{
  obs = obsIn;
}

void PCLView::resetViewport()
{
//  viewer->initCameraParameters();
//  viewer->resetCamera(); // translation
//  viewer->setCameraPosition(5.0,5.0,5.0,10.0,10.0,10.0,1.0,1.0,1.0,0);
//  viewer->setCameraPosition(-29.570503, -52.226951, 51.029257, 0.540905, 0.478015, 0.692043);
//  viewer->setCameraPosition(0, 0, 100, 0.0,0.0,1);
//  viewer->setCameraPosition(0.0,0.0,100.0,0.0,0.8,0.6);
  displayCloud();
}
