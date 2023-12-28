#include "mainwindow.h"

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent)
{
  setGeometry(0, 0, 1650, 800);
  setWindowTitle("点云分析");
  setWindowIcon(QIcon("/media/jpw/m10/c-qt/asserts/logo.ico"));
  std::cout << "create main window" << std::endl;
}

MainWindow::~MainWindow()
{
  std::cout << "delete main window" << std::endl;
}