#ifndef PANEL_H
#define PANEL_H

#include <QEasingCurve>
#include <QObject>
#include <QPainter>
#include <QString>
#include <QWidget>

#include <complex>

class Panel : public QWidget
{
  Q_OBJECT
 public:
  Panel();
  Panel(QWidget *parent);
  ~Panel();
  void setTitle(QString titleIn);
 public Q_SLOTS:
  void setRange(float minValued, float maxValued);
  void setUint(QString unitIn);
  void setCurrentValue(float valueIn = 30);
  void setStep(int stepIn); // num of intervals(gaps) between divides; 刻度线形成的间隔数量

 signals:

 private slots:

 private:
  void initPanel();
  void drawFrame(QPainter& painter); // 表盘背景; background
  void drawNumberIndicator(QPainter& painter); // 显示当前速度数字; display current number
  void drawDividing(QPainter& painter); // 刻度
  void drawIndicator(QPainter& painter); // 指针
  void drawNumberValue(QPainter& painter); // 表盘速度数字; draw number in panel
  void paintEvent(QPaintEvent *noevent);

  float startAngle = 135, endAngle = 45; // angle range of divides; 刻度的起止角度
  float refSize = 200, radius = 100;
  float minValue = 0, maxValue = 100, curValue = 10, trueValue;
  // divide points to center of circle, so there is an angkle between divide and horizon
  float anglePerValue; // angle between neighbor divides
  float stepSize; // value differnece between neighbor divides
  float angleStep; // angle of one divide
  int step = 25; // 刻度线形成的间隔数量
  QString unit = "km/h";
  QString title = "Value";

};

/*
 *  pp = new Panel(ui->imgLab);
 *  pp->move(100,100);
*/
#endif // PANEL_H
