#ifndef AZIMUTHPANEL_H
#define AZIMUTHPANEL_H

#include <QDebug>
#include <QEasingCurve>
#include <QObject>
#include <QPushButton>
#include <QPainter>
#include <QString>
#include <QWidget>

#include <complex>

static const QPoint hourHand[3] =
  {
    QPoint(10, 0),
    QPoint(-10, 0),
    QPoint(0,-100)
};

class AzimuthPanel : public QWidget
{
  Q_OBJECT
public:
  AzimuthPanel();
  AzimuthPanel(QWidget *parent);
  void initAzimuthPanel();
  ~AzimuthPanel();

  int angle;
  QColor red;
  QColor mainColor;

public Q_SLOTS:
  void setCurrentValue(float valueIn);

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

//  float startAngle, endAngle;
//  float refSize, radius;
  float minSpeed, maxSpeed, curSpeed, trueSpeed;
//  float anglePerSpeed, stepSize, angleStep;
  int step = 18;
  int W,H,side;

};

#endif // AZIMUTHPANEL_H
