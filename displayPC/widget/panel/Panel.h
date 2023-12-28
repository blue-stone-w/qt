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
  // change range
  void setRange(float minValued, float maxValued);
  void setUint(QString unitIn);
  void setCurrentValue(float valueIn);
  void initPanel();
  void setStep(int stepIn);

 signals:

 private slots:

 private:
  void drawFrame(QPainter& painter);
  void drawNumberIndicator(QPainter& painter);
  void drawDividing(QPainter& painter);
  void drawIndicator(QPainter& painter);
  void drawNumberSpeed(QPainter& painter);
  void paintEvent(QPaintEvent *noevent);

  float startAngle, endAngle;
  float refSize, radius;
  float minSpeed, maxSpeed, curSpeed, trueSpeed;
  float anglePerSpeed, stepSize, angleStep;
  int step = 25;
  QString unit = "km/h";
  QString title = "speed";

};

/*
 *  pp = new Panel(ui->imgLab);
 *  pp->move(100,100);
*/
#endif // PANEL_H
