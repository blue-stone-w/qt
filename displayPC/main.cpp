#include "mainwindow.h"
#include <QApplication>
#include <QObject>
#include <QProcess>

#include <global.h>
#include <SavePcd.h>
#include <string>


std::thread *rosThread;
QSettings *settings; //申明一个QSetting类函数

void runRos()
{
  ros::NodeHandle mnh; // !!! main node handle: if no other node handle is created, this node handle will keep program going.

  std::cout << "have started ros" << std::endl << std::endl;

  SavePcd *savepcd;
  if(settings->value("/ros/save").toBool())
  {
    savepcd = new SavePcd("/whole_cloud");
    savepcd->setPath(settings->value("/ros/path").toString().toStdString());
    savepcd->setAff(0.0, 0.0, 0.0, 0.0, 0.0, 0.0); // 7.3, -0.15, 2.1, 0.0, 0.20944, 0.0  ;  -8.04, 0.0, 0.94, 0.0, 0.0, 3.1415927

    std::cout << "- ros function: save" << std::endl;
    std::cout << " - save path:" << settings->value("/ros/path").toString().toStdString() << std::endl << std::endl;
  }

  // you can add more ros function here

  ros::Rate frequence(50);
  while(ros::ok())
  {
    ros::spinOnce();
    frequence.sleep();
  }
  return;
}

int main(int argc, char *argv[])
{
  QApplication a(argc, argv);
  MainWindow w;
  w.show();
  w.OutPut("have created main window");

  settings = new QSettings ("./config/config.ini", QSettings::IniFormat); //构建函数
  w.OutPut("have read configuration");

  // start roscore
  if(settings->value("/ros/ros").toBool())
  {
    system("gnome-terminal -- bash -c 'roscore'&");
    sleep(1);
    ros::init(argc, argv, "roscode");
    sleep(1);
    rosThread = new std::thread(&runRos);
    rosThread->detach();
    w.threads.push_back( rosThread->native_handle() );
  }

  return a.exec();
}
