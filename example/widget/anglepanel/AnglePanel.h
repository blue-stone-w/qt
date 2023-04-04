#ifndef ANGLEPANEL_H
#define ANGLEPANEL_H

#include <QDebug>
#include <QEasingCurve>
#include <QObject>
#include <QPushButton>
#include <QPainter>
#include <QString>
#include <QWidget>

#include <complex>

class AnglePanel : public QWidget
{
  Q_OBJECT
 public:
  AnglePanel();
  AnglePanel(QWidget *parent);
  ~AnglePanel();
  QPushButton anticlockwise, clockwise, numupdate;
  int curAngle, temAngle;
 public Q_SLOTS:
  // change range
  void setRange(float minValued, float maxValued);
  void setCurrentValue(float valueIn);
  void initAnglePanel();
  void setStep(int stepIn);

 signals:

 private slots:

 private:
  // 外圈的彩色圆圈
  void drawFrame(QPainter& painter);
  // 刻度线的数字
  void drawNumberIndicator(QPainter& painter);
  // 刻度线
  void drawDividing(QPainter& painter);
  // 指针
  void drawIndicator(QPainter& painter);
  // 数字
  void drawNumberSpeed(QPainter& painter);
  // 收到新的数据后，重新绘制整个表盘
  void paintEvent(QPaintEvent *noevent);

  float startAngle, endAngle;
  float refSize, radius;
  float minSpeed, maxSpeed, curSpeed, trueSpeed;
  float anglePerSpeed, stepSize, angleStep;
  int step = 18;

};

/*
 *  pp = new AnglePanel(ui->imgLab);
 *  pp->move(100,100);
*/
#endif // ANGLEPANEL_H
