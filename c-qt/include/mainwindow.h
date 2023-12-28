#include <iostream>

#include <QMainWindow>

namespace Ui
{
  class MainWindow;
}

class MainWindow : public QMainWindow
{
  // 本质是一个已定义好的宏，所有需要“信号和槽”功能的组件都必须将 Q_OBJECT 作为 private 属性成员引入到类中
  Q_OBJECT

public:
  explicit MainWindow(QWidget *parent = 0);

  ~MainWindow();

  // private:
  // Ui::MainWindow *ui;
};