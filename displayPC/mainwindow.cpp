#include "mainwindow.h"

// warn no files;save merged cloud; cache clouds; stop when select a cloud; delete cloud;
MainWindow::MainWindow(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::MainWindow)

{
  ui->setupUi(this);
//  this->setGeometry(0,0,1650,800);
  this->setWindowIcon(QIcon("./source/logo.ico"));
//  this->setWindowTitle("点云分析");

  settings = new QSettings ("./config/config.ini", QSettings::IniFormat); //构建函数
  if(settings->value("/perception/algorithm").toString() == QString("patchwork"))
  {
    OutPut("perception: patchwork");
    perception = new PatchWorkBase();
  }
  else if(settings->value("/perception/algorithm").toString() == QString("covariance"))
  {
    OutPut("perception: Covariance");
    perception = new CovarianceBase();
  }
  else
  {
    OutPut(settings->value("/perception/algorithm").toString() + QString(" doesn't exist! Now perception: patch work"));
    perception = new PatchWorkBase();
  }

  //--- display image and pcd cloud ---//
  pcdView.setParent(ui->pcdGroup);

  //--- file list ---//

  //--- display state ---//
  ui->displayOriCloud2->setToggle(true);
  filterSize2 = new EditableLabel();
  ui->stateLay->addWidget(filterSize2,2,8);
//  ui->time2->setText(QDateTime::fromMSecsSinceEpoch(long long int msecs));
  ui->time2->setText(QDateTime::currentDateTime().toString("MM-dd hh:mm:ss.zzz"));

  int numtem = pcdView.cloudSize();
  ui->cloudSize2->setText(QString::number(numtem/10000,10) + QString(",") + QString::number(numtem%10000,10));

  sumFiles = QString(" / ")+QString::number(0,10);
  ui->indexpcd2->setText(QString::number(0,10)+sumFiles);

  // -- play setting ---//
  playIco = QIcon("./source/play.png");
  pauseIco = QIcon("./source/pause.png");

  connect(ui->playOne, &QPushButton::clicked, this, &MainWindow::playPause);
  connect(ui->lastOne, &QPushButton::clicked, this, &MainWindow::lastFrame);
  connect(ui->nextOne, &QPushButton::clicked, this, &MainWindow::nextFrame);

  connect(&playTimer, SIGNAL(timeout()), this, SLOT(playClouds()));
  connect(ui->addBut, &QPushButton::clicked, this, &MainWindow::addInterval);
  connect(ui->minusBut, &QPushButton::clicked, this, &MainWindow::minusInterval);

  connect(ui->selectpath, SIGNAL(triggered()), this, SLOT(openDirectory()));
  connect(ui->about, SIGNAL(triggered()), this, SLOT(openAbout()));

  parseWin = new ParseDataForm();
   parseWin->setWindowTitle("远程操作");
}

MainWindow::~MainWindow()
{
  delete ui;
}
void MainWindow::playClouds()
{
  if(curIndex < pathVector.size() - 1)
  {
    curIndex++;
    ui->fileList->clearSelection();
    ui->fileList->item(curIndex)->setSelected(true);
    updateAll();
  }
  else
  {
    ui->playOne->setIcon(pauseIco);
    isPlay = false;
    playTimer.stop();
  }
}

void MainWindow::playPause()
{
  if(isPlay)
  {
    isPlay = false;
    playTimer.stop();
    ui->playOne->setIcon(pauseIco);
    return;
  }
  else
  {
    if(curIndex < pathVector.size() - 1)
    {
      ui->playOne->setIcon(playIco);
      isPlay = true;

      playTimer.start(ui->gap->text().toFloat()*1000);
    }
    return;
  }
}

void MainWindow::lastFrame()
{
  isPlay = false;
  playTimer.stop();
  ui->playOne->setIcon(pauseIco);
  if(curIndex > 0)
  {
    curIndex--;
    ui->fileList->clearSelection();
    ui->fileList->item(curIndex)->setSelected(true);
    updateAll();
  }
}

void MainWindow::nextFrame()
{
  isPlay = false;
  playTimer.stop();
  ui->playOne->setIcon(pauseIco);
  if(curIndex < pathVector.size() - 1)
  {
    curIndex++;
    ui->fileList->clearSelection();
    ui->fileList->item(curIndex)->setSelected(true);
    updateAll();
  }
}

void MainWindow::addInterval()
{
  int interval = round(ui->gap->text().toFloat()*10);
  if(interval < 10)
  {
    ui->gap->setText(QString::number(float(interval + 1) / 10));
  }
}

void MainWindow::minusInterval()
{
  int interval = round(ui->gap->text().toFloat()*10);
  if(interval > 1)
  {
    ui->gap->setText(QString::number(float(interval - 1) / 10));
  }
}

bool MainWindow::openDirectory()
{
  pathVector.clear();
  ui->fileList->clear();
  curIndex = 0;

  QString dirpath = QFileDialog::getExistingDirectory(this, "选择目录", QString(std::getenv("HOME")) +  "/temp/", QFileDialog::ShowDirsOnly);
  QDir dir(dirpath);
  dir.setFilter(QDir::Files | QDir::Hidden | QDir::NoSymLinks);
  dir.setSorting(QDir::Name);

  QFileInfoList list = dir.entryInfoList();

  for (int i = 0; i < list.size(); ++i)
  {
    QFileInfo fileInfo = list.at(i);
    if(fileInfo.fileName().endsWith(".pcd"))
    {
      pathVector.push_back(QPair<QString,QString>(fileInfo.filePath(),fileInfo.fileName().left(fileInfo.fileName().size()-4)));
    }
  }
  if(pathVector.empty())
  {
    return false;
  }
  for (int i = 0; i < pathVector.size(); ++i)
  {
    ui->fileList->addItem(pathVector[i].second);
  }
  updateAll();
  pcdView.resetViewport();

  sumFiles = QString(" / ")+QString::number(pathVector.size(),10);
  update(); // necessary for slot
  return true;
}


void MainWindow::on_fileList_clicked(const QModelIndex &index)
{
  curIndex = index.row();
  updateAll();
}

void MainWindow::updateAll()
{

  pcl::PointCloud<pcl::PointXYZI>::Ptr thisCloud;
  thisCloud.reset(new pcl::PointCloud<pcl::PointXYZI>);


  if(-1 == pcl::io::loadPCDFile(pathVector[curIndex].first.toStdString(), *thisCloud))
  {
    QMessageBox MyBox(QMessageBox::Warning,"警告","点云文件不存在",QMessageBox::No|QMessageBox::Yes);
    MyBox.exec();
  }

//  std::cout << settings->value("/perception/perception").toBool() << ", " << ui->switcherDual->isToggled() << std::endl;

  updateDisplay(thisCloud);

  ui->time2->setText(QDateTime::fromMSecsSinceEpoch(pathVector[curIndex].second.toLong()).toString("MM-dd hh:mm:ss.zzz"));
  ui->indexpcd2->setText(QString::number(curIndex+1,10)+sumFiles);
  int numtem = pcdView.cloudSize();
  ui->cloudSize2->setText(QString::number(numtem/10000,10) + QString(",") + QString::number(numtem%10000,10));
}

void MainWindow::updateDisplay(pcl::PointCloud<pcl::PointXYZI>::Ptr oriCloud)
{
  if(settings->value("/perception/perception").toBool() ) // load when startup
  {
    double time = pathVector[curIndex].second.toDouble() / 1000;
    perception->runPerception(*oriCloud, time);
  }

  pcdView.inputOriCloud(ui->displayOriCloud2->isToggled() ? perception->parseOriCloud() : pcl::PointCloud<pcl::PointXYZI>());

  pcdView.inputObsCloud(ui->displayObsCloud2->isToggled() ? (perception->parseObjectCloud()) : pcl::PointCloud<pcl::PointXYZI>());

  pcdView.inputObs(ui->displayObs2->isToggled() ? (perception->parseObs()) : std::vector<ObjectBox>());
//std::cout << "box num: " << perception->parseObs().size() << std::endl;
  ui->obsnum2->setText(QString::number(perception->parseObs().size(),10));

  pcdView.displayCloud();
}

void MainWindow::on_mergepcd_clicked()
{
  QItemSelectionModel *selections = ui->fileList->selectionModel(); // 返回当前的选择模式
  QModelIndexList selectedsList = selections->selectedIndexes(); // 返回所有选定的模型项目索引列表

  if(!selectedsList.count())
  {
    return;
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloudstem;
  for (int i = 0; i < selectedsList.count(); i++)
  {
    pcl::PointCloud<pcl::PointXYZI> cloudtem;
    pcl::io::loadPCDFile(pathVector[selectedsList.at(i).row()].first.toStdString(), cloudtem);
    *cloudstem += cloudtem;
  }

  updateDisplay(cloudstem);

  int numtem = pcdView.cloudSize();
  ui->cloudSize2->setText(QString::number(numtem/10000,10) + QString(",") + QString::number(numtem%10000,10));
}

void MainWindow::on_filterpcd_clicked()
{
  pcdView.filterCloud(filterSize2->label->text().toFloat());
  int numtem = pcdView.cloudSize();
  ui->cloudSize2->setText(QString::number(numtem/10000,10) + QString(",") + QString::number(numtem%10000,10));
}

void MainWindow::closeEvent(QCloseEvent *event)
{
  Q_UNUSED(event);
  qDebug() << "";
  for( int i = 0, size = threads.size();  i < size; i++)
  {
     qDebug() << "close thread" << pthread_cancel(threads[i]);
  }

  sleep(1);

}

void MainWindow::on_viewport_clicked()
{
    pcdView.resetViewport();
}

void MainWindow::on_pushButton_3_clicked()
{

  QProcess process;
  QStringList args;
  args << "/home/jpw/rec.sh";

  if(!QFileInfo(args.at(0)).isFile())
  {
//    qDebug() << "script doesn't exist";
    QString temStr = QString("Record script doesn't exist!");
    OutPut(temStr);

    return;
  }

  qDebug() << "start";
  process.start("sh", args);
  sleep(5);
  process.close();
  qDebug() << "close";
//  system("gnome-terminal -- bash -c 'echo 123456 | -S ssh nnnn@10.10.1.14 ;roscore;'&"); cd ~;rosbag record -a; cd ~;ssh nnnn@10.10.1.14
//  system("gnome-terminal -- bash -c 'ssh nnnn@10.10.1.14;'");
}

void MainWindow::OutPut(QString out)
{
  QString temStr = QString::number(outsize++) + QString(": ") + out;
  outStr.push_back(temStr);
  ui->output->append(temStr);
  if(outStr.size() > 200) // max outputs
  {
    ui->output->clear();
    outStr.erase(outStr.begin(), outStr.end()-100); // reserve some new outputs
    for(int i = 0; i < outStr.size(); i++)
    {
      ui->output->append(outStr[i]);
    }
  }
}


void MainWindow::on_parseData_clicked()
{

//  ParseDataForm *parseWin = new ParseDataForm();
//  parseWin->setWindowTitle("远程操作");
  parseWin->show();
}

bool MainWindow::openAbout()
{
  QMessageBox::about(this,"关于本软件","用于调试感知模块的工具。集成了感知模块的程序和数据的获取，包括数据的远程录制和本地解析。");
  return true;
}

/* todo: net connection indicator; reset camera and view
*/
