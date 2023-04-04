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
  //本质是一个已定义好的宏，所有需要“信号和槽”功能的组件都必须将 Q_OBJECT 作为 private 属性成员引入到类中
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
