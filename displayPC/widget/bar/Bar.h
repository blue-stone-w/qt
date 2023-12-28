#ifndef BAR_H
#define BAR_H

#include <QObject>
#include <QLabel>
#include <QWidget>
#include <QPainter>
#include <QString>
#include <QDebug>

class Bar : public QWidget
{
  Q_OBJECT
 public:
  Bar();
  Bar(QWidget *parent);
  ~Bar();
  void setCurrValue(int valueIn = 0);
  void resize(int w, int h);
  void setGeometry(int x, int y, int w, int h);
protected:


 signals:

 private:
  QLabel *background, *colorbar;
  int px,py,width,height;
  int currPentcent;
  QWidget *win;

};
/*
 * bb = new Bar(ui->imgLab);
 * bb->setGeometry(10,500,500,50);
 * bb->setCurrValue(26);
*/
#endif // BAR_H
