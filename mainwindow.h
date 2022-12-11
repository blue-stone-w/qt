#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <Bar.h>
#include <Battery.h>
#include <EditableLabel.h>
#include <InfoLabel.h>
#include <Panel.h>
#include <SwitcherDual.h>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(QWidget *parent = 0);
  ~MainWindow();

private:
  Ui::MainWindow *ui;
  Bar bar;
  Battery battery;
  EditableLabel editable_label;
  InfoLabel info_label;
  Panel panel;
  SwitcherDual switch_dual;
};

#endif // MAINWINDOW_H
