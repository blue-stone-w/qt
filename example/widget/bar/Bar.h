#ifndef BAR_H
#define BAR_H

#include <QObject>
#include <QLabel>
#include <QWidget>
#include <QPainter>
#include <QString>
#include <QDebug>

/*
This Bar is consisted by background and changeable bar. Background is lower layer.
*/

class Bar : public QWidget
{
  Q_OBJECT
 public:
  Bar();
  Bar(QWidget *parent);
  ~Bar();
  bool update(float valueIn = 30);
  void setGeometry(int x = 10, int y = 350, int w = 500, int h = 50); // set position and size of background
  void setRange(float minIn = 0, float maxIn = 100);
 protected:

 signals:

 private:
  void setCurrValue(float valueIn = 30);
  void draw();
  void resize(int w = 300, int h = 50); // change size of bar according current value
  QLabel *background, *colorbar; // background color; bar colar
  int px,py,width,height; // position and size
  int currPentcent; // calculate current/range to get width of bar
  float min, max, range;
  QWidget *win; // parent widget

};
#endif // BAR_H
