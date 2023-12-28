#ifndef PARSEDATAFORM_H
#define PARSEDATAFORM_H

#include "./remote/truckparam.h"

#include "yaml-cpp/yaml.h"

#include <iostream>
#include <fstream>

#include <QCoreApplication>
#include <QMap>
#include <QMessageBox>
#include <QString>
#include <QVector>
#include <QWidget>

//#include "ui_mainwindow.h"
//#include "mainwindow.h"

namespace Ui {
class ParseDataForm;
}

class ParseDataForm : public QWidget
{
  Q_OBJECT

public:
  explicit ParseDataForm(QWidget *parent = 0);
  ~ParseDataForm();

private slots:
  void on_comboBox_currentIndexChanged(int index);

  void on_saveButton_clicked();

private:
  QMap<std::string,TruckParam> truck_paramm;
  QVector<TruckParam> truck_paramv;
  YAML::Node config;
  Ui::ParseDataForm *ui;
  bool init();
  bool initTruckParam();
//  bool updateIndex(int index);
};

#endif // PARSEDATAFORM_H
