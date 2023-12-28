#include "Panel.h"

Panel::Panel(): QWidget()
{
  Panel::initPanel();
}

Panel::Panel(QWidget *parent): QWidget(parent)
{
  Panel::initPanel();
}

Panel::~Panel() {}

void Panel::initPanel()
{
  startAngle = 135, endAngle = 45;
  refSize = 200, radius = refSize/2; // reference size
  minSpeed = 0, maxSpeed = 100, curSpeed = 0;
  anglePerSpeed = (360-(startAngle-endAngle)) / (maxSpeed-minSpeed);
  this->setFont(QFont("SimSun",10));
  this->setRange(0,100);
  setWindowFlags(Qt::FramelessWindowHint);
  setAttribute(Qt::WA_TranslucentBackground);

  this->resize(refSize,refSize);
}

void Panel::paintEvent(QPaintEvent *noevent)
{
  Q_UNUSED(noevent);
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing); // anti-aliasing
//  float scale = width();
  float scale = 200;
  // note order of setting scale and origin
  painter.scale(scale/refSize, scale/refSize);
  painter.translate(refSize/2,refSize/2); // set origin's position
  angleStep = (360.0 - (startAngle-endAngle)) / step;
  stepSize = (maxSpeed-minSpeed) / step;
  drawFrame(painter); // panel
  drawDividing(painter); //
  drawNumberIndicator(painter); //
  drawIndicator(painter); // speed pointer
  drawNumberSpeed(painter);
}

void Panel::drawFrame(QPainter& painter)
{
  painter.save();
  painter.setPen(Qt::NoPen); // fill without boundary

  // draw gray round frame
  QLinearGradient lg1(-radius, -radius, radius, radius);
  lg1.setColorAt(0,Qt::gray);
  lg1.setColorAt(0.5,Qt::gray);
  lg1.setColorAt(1,Qt::gray);
  painter.setBrush(lg1);
  painter.drawEllipse(-radius, -radius, refSize, refSize);

  //
  QConicalGradient conical(0,0,-45); // (x, y, start angle)
  conical.setColorAt(0,Qt::red);
  conical.setColorAt(0.3,Qt::yellow);
  conical.setColorAt(0.75,Qt::green);
  conical.setColorAt(1,Qt::white);

  painter.setPen(Qt::transparent);
  painter.setBrush(conical);
  painter.drawEllipse(QPoint(0,0), 95,95); // radius is 100, this circle is little smaller than gray circle

  QColor backgroundColor = QColor(255,250,250);

  // white cover down half circle
  QPen pen;
  pen.setCapStyle(Qt::FlatCap);
  pen.setWidthF(10);
  pen.setColor(backgroundColor);
  painter.setPen(pen);
  QRectF rect = QRectF(-90,-90,90*2,90*2); // left-right position, size
  painter.drawArc(rect,-45*16,-90*16); // (,start angle, angle span)

  // white circle cover center
  painter.setBrush(backgroundColor);
  painter.drawEllipse(QPoint(0,0),81,81);

  painter.restore();

}

void Panel::drawDividing(QPainter &painter)
{
  painter.save();
  painter.rotate(startAngle);

  QPen pen(Qt::black);
  painter.setPen(pen);

  for(int i = 0; i <= step; i++)
  {
    if(i >= 20)
    {
      pen.setColor(Qt::red);
      painter.setPen(pen);
    }

    if(i%5 == 0) // thick line
    {
      pen.setWidth(2);
      painter.setPen(pen);
      painter.drawLine(88,0,75,0); // (88,0)(75,0)two points in this line segment
    }
    else if(i%10 == 0) // middle line
    {
      pen.setWidth(1);
      painter.setPen(pen);
      painter.drawLine(88,0,80,0); // two points in this line segment
    }
    else if(i%1 == 0) // middle line
    {
      pen.setWidth(0);
      painter.setPen(pen);
      painter.drawLine(83,0,80,0); // two points in this line segment
    }

    painter.rotate(angleStep);
  }
  painter.restore();

}

void Panel::drawNumberIndicator(QPainter &painter)
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
    QString speed = QString::number(minSpeed + i*stepSize);

    if(i%5 == 0)
    {
      w = fm.width(speed);
      h = fm.height();
      painter.drawText(QPointF(x-w/2, y+h/4),speed);
    }
  }
  painter.restore();
}

void Panel::drawIndicator(QPainter &painter)
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
  rg.setColorAt(1.0,Qt::darkGray);
  painter.setPen(Qt::NoPen);
  painter.setBrush(rg);
  painter.drawEllipse(QPoint(0,0),10,10);
  painter.restore();
}

void Panel::drawNumberSpeed(QPainter &painter)
{
  painter.save();
  painter.setPen(Qt::black);
  QString speed = QString("%1").arg(curSpeed);

  QFontMetricsF fm(this->font());
  qreal w = fm.size(Qt::TextSingleLine,speed).width();
  painter.drawText(-w/2,30,speed);

  QFontMetricsF fm1(this->font());
  qreal w1 = fm1.size(Qt::TextSingleLine,unit).width();
  painter.drawText(-w1/2,55,unit);

  QFontMetricsF fm2(this->font());
  qreal w2 = fm2.size(Qt::TextSingleLine,title).width();
  painter.drawText(-w2/2,80,title);

  painter.restore();
}

void Panel::setRange(float minValued, float maxValued)
{
  minSpeed = minValued;
  maxSpeed = maxValued;
  anglePerSpeed = (360.0 - (startAngle-endAngle)) / (maxSpeed-minSpeed);
  update();
}

void Panel::setUint(QString unitIn)
{
  this->unit = unitIn;
}

void Panel::setCurrentValue(float valueIn)
{
  curSpeed = valueIn;
  update();
}

void Panel::setTitle(QString titleIn)
{
  this->title = titleIn;
}

void Panel::setStep(int stepIn)
{
  step = stepIn;
}

// todo:
