#include "AnglePanel.h"

AnglePanel::AnglePanel(): QWidget()
{
  AnglePanel::initAnglePanel();
}

AnglePanel::AnglePanel(QWidget *parent): QWidget(parent)
{
  AnglePanel::initAnglePanel();
}

AnglePanel::~AnglePanel() {}

void AnglePanel::initAnglePanel()
{
  startAngle = 180, endAngle = 0;
  refSize = 200, radius = refSize/2; // reference size
  minSpeed = -90, maxSpeed = 90, curSpeed = 0;
  anglePerSpeed = (360-(startAngle-endAngle)) / (maxSpeed-minSpeed);
  this->setFont(QFont("SimSun",10));
  setWindowFlags(Qt::FramelessWindowHint);
  setAttribute(Qt::WA_TranslucentBackground);

  this->resize(refSize,refSize);


}

void AnglePanel::paintEvent(QPaintEvent *noevent)
{
  Q_UNUSED(noevent);
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing); // anti-aliasing
//  float scale = width();
  float scale = 200;
  // note order of setting scale and origin
  painter.scale(scale/refSize, scale/refSize);
  painter.translate(refSize/2,refSize/2); // set origin's position
  angleStep = (360.0 - (startAngle-endAngle)) / step; // default 30
  stepSize = (maxSpeed-minSpeed) / step; // default 30

  drawFrame(painter); // AnglePanel
  drawDividing(painter); //
  drawNumberIndicator(painter); //
  drawIndicator(painter); // speed pointer
  drawNumberSpeed(painter);
}

void AnglePanel::drawFrame(QPainter& painter)
{
  painter.save();
  painter.setPen(Qt::NoPen); // fill without boundary

  // draw gray round frame
//  QLinearGradient lg1(-radius, -radius, radius, radius);
//  lg1.setColorAt(0,Qt::gray);
//  lg1.setColorAt(0.5,Qt::gray);
//  lg1.setColorAt(1,Qt::gray);
//  painter.setBrush(lg1);
//  painter.drawEllipse(-radius, -radius, refSize, refSize);
  painter.setPen(Qt::gray);
  QRectF rectangle(-100,-100, 200.0, 200);
  int startAngle = 0;//起始角度，角度可以为负值，如-30*16
  int spanAngle = 180*16;//覆盖的角度，绘制方向为逆时针方向
  painter.drawRect(rectangle);
  painter.drawPie(rectangle, startAngle, spanAngle);


  QColor backgroundColor = QColor(255,250,250);

  // white circle cover center
//  painter.setBrush(backgroundColor);
//  painter.drawEllipse(QPoint(0,0),97,97);

  painter.restore();

}

void AnglePanel::drawDividing(QPainter &painter)
{
  painter.save();
  painter.rotate(startAngle);

  QPen pen(Qt::black);
  painter.setPen(pen);

  for(int i = 0; i <= step; i++)
  {
    if(i%3 == 0) // thick line
    {
      painter.setPen(Qt::blue);
      pen.setWidth(2);
      painter.drawLine(97,0,87,0); // (88,0)(75,0)two points in this line segment
    }
    else // middle line
    {
      painter.setPen(Qt::black);
      pen.setWidth(1);
      painter.drawLine(97,0,90,0); // two points in this line segment
    }

    painter.rotate(angleStep);
  }
  painter.restore();

}

void AnglePanel::drawNumberIndicator(QPainter &painter)
{
  painter.save();

  painter.setPen(Qt::black);

  double x,y;
  double angle, angleArc;
  double w,h;
  QFontMetricsF fm(this->font());

  for(int i = 0; i <= step; i++)
  {
    angle = 360 - (startAngle + i*angleStep);
    angleArc = angle * 3.14/180;
    x = 65 * cos(angleArc);
    y = -65 * sin(angleArc);
    QString speed = QString::number(abs(minSpeed + i*stepSize));


    if(i%3 == 0)
    {
      w = fm.width(speed);
      h = fm.height();
      painter.drawText(QPointF(x-w/2, y+h/4),speed);
    }
  }
  painter.restore();
}

void AnglePanel::drawIndicator(QPainter &painter)
{
  painter.save();
  float curSpeedtemp;
  if(curSpeed < minSpeed)
  {
    curSpeedtemp = minSpeed;
  }
  else if(curSpeed > maxSpeed)
  {
    curSpeedtemp = maxSpeed;
  }
  else
  {
    curSpeedtemp = curSpeed;
  }
  double curAngle = startAngle + (curSpeedtemp-minSpeed)*anglePerSpeed;
  painter.rotate(curAngle);
  QRadialGradient haloGradient(0,0,60,0,0); // (centre of circle, radius, start position)
  haloGradient.setColorAt(0,QColor(200,60,60));
  haloGradient.setColorAt(1,QColor(0,0,0)); // gray
  painter.setPen((Qt::red)); // color of line and text
  painter.setBrush(haloGradient); // how to fill this area

  static const QPointF points[3] = { QPointF(0.0,3), QPointF(0.0,-3), QPointF(60,0) };
  painter.drawPolygon(points,3);
  painter.restore();

  // draw rotational center
  painter.save();
  QRadialGradient rg(0,0,60,0,0);
  rg.setColorAt(0.0,Qt::darkGray);
  rg.setColorAt(0.5,Qt::white);
  rg.setColorAt(1.0,Qt::white);
  painter.setPen(Qt::NoPen);
  painter.setBrush(rg);
  painter.drawEllipse(QPoint(0,0),15,15);
  painter.restore();
}

void AnglePanel::drawNumberSpeed(QPainter &painter)
{
  painter.save();
  painter.setPen(Qt::green);
  QString speed = QString("%1").arg(curSpeed);

  QFontMetricsF fm(this->font());
  qreal w = fm.size(Qt::TextSingleLine,speed).width();
  painter.drawText(-w/2,5,speed);

  painter.restore();
}

void AnglePanel::setRange(float minValued, float maxValued)
{
  minSpeed = minValued;
  maxSpeed = maxValued;
  anglePerSpeed = (360.0 - (startAngle-endAngle)) / (maxSpeed-minSpeed);
  update();
}

void AnglePanel::setCurrentValue(float valueIn)
{
  curSpeed = valueIn;
  update();
}


void AnglePanel::setStep(int stepIn)
{
  step = stepIn;
}

// todo:
