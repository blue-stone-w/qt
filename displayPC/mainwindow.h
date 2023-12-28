#ifndef MAINWINDOW_H
#define MAINWINDOW_H


// qt
#include <QDateTime>
#include <QDebug>
#include <QFileDialog>
#include <QMessageBox>
#include <QProcess>
#include <QSettings>
#include <QString>
#include <QVector>
#include <QWidget>


// C++
#include <thread>
#include <vector>

// pcl
#include <pcl/io/pcd_io.h>


#include "ui_mainwindow.h"
#include "parsedataform.h"

#include <QMainWindow>

#include "widget/panel/Panel.h"
#include "widget/bar/Bar.h"
#include "widget/editablelabel/EditableLabel.h"

#include "viewer/pclview/pclview.h"
#include "myprocess/perception/patchwork/patchworkbase.h"
#include "myprocess/perception/covariance/covariancebase.h"

namespace Ui
{
class MainWindow;
}

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(QWidget *parent = 0);
  ~MainWindow();

  //--- thread ---//
  std::vector<pthread_t> threads;
  void OutPut(QString out);

private:
  Ui::MainWindow *ui;

  ParseDataForm *parseWin;

  QSettings *settings; //申明一个QSetting类函数

  int outsize = 1;

  //--- image and pcd cloud ---//
  PCLView pcdView;

  PerceptionInterface *perception;

  //--- state ---//
  QString sumFiles; // update after selecting directory
  EditableLabel *filterSize2;

  // -- play ---//
  QIcon playIco, pauseIco;
  bool isPlay = false;
  QTimer playTimer;
  void playPause();     //
  void lastFrame();     //
  void nextFrame();     //
  void addInterval();   // add interval between two cloud when we play cloud list
  void minusInterval(); //
  private slots:
   void playClouds();

  //--- file ---//
 private:
  QVector<QPair<QString,QString>> pathVector;
  QVector<QString> outStr;
  int curIndex = 0;
  void updateAll(); // update viewer, info text
  void updateDisplay(pcl::PointCloud<pcl::PointXYZI>::Ptr oriCloud);
  private slots:
   bool openDirectory();
   bool openAbout();
 private slots:
  void on_fileList_clicked(const QModelIndex &index); // choose a cloud to process and display
  void on_mergepcd_clicked();  // use only when cloud is in global cooridate system
  void on_filterpcd_clicked(); //

  void on_viewport_clicked();

  void on_pushButton_3_clicked();

  void on_parseData_clicked();

protected:
  void closeEvent(QCloseEvent *event) override;
};

#endif // MAINWINDOW_H


/*
playStyle = "border: 1px solid;";
*/
