#include <iostream>

#include <QtWidgets/QDialog>
#include <QtWidgets/QApplication>
#include <QMainWindow>

#include <QDialog>
#include <QLabel>
#include "mainwindow.h"
int main(int argc, char *argv[])
{
  std::cout << "o" << std::endl;
  QApplication a(argc, argv);
  MainWindow w;
  w.show();

  return a.exec();
}