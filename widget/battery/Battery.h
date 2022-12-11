#ifndef BATTERY_H
#define BATTERY_H

#include <QColor>
#include <QDebug>
#include <QPainter>
#include <QPen>
#include <QRect>
#include <QWidget>

class Battery : public QWidget
{
  Q_OBJECT
 public:
  Battery(QWidget *parent = nullptr);
  ~Battery();
  void setPower(int powerIn);
  int getPower();
  void setWarnLevel(int warnLevelIn);
  int getWarnLevel();
  QSize sizeHint(); // default size
  void setParent(QWidget *parent);

 signals:

 public slots:
 private:
  QColor colorBackground = Qt::white;
  QColor colorBorder = Qt::black;
  QColor colorPower = Qt::green;
  QColor colorWarning = Qt::red;
  int power = 60; // 0 - 100
  int warnLevel = 20;
 protected:
  void paintEvent(QPaintEvent *event) Q_DECL_OVERRIDE; // override a virtual function

};

#endif // BATTERY_H
