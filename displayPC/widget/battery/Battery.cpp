#include "Battery.h"

Battery::Battery() : QWidget()
{
  this->resize(125,55);
}

Battery::Battery(QWidget *parent) : QWidget(parent)
{
  this->resize(125,55);
}

Battery::~Battery() { }

//void Battery::setParent(QWidget *parent)
//{
//  this->setParent(parent);
//}

void Battery::paintEvent(QPaintEvent *event)
{
  // avoid compilation warning
  Q_UNUSED(event);
  QPainter painter(this);
  QRect rect(0,0,width(),height());

  painter.setViewport(rect);
  painter.setWindow(0,0,120,50);

  painter.setRenderHint(QPainter::Antialiasing);
  painter.setRenderHint(QPainter::TextAntialiasing);

  QPen pen;
  pen.setWidth(2);
  pen.setColor(colorBorder);
  pen.setStyle(Qt::SolidLine);
  pen.setCapStyle(Qt::FlatCap); // 用来指定线的两端样式，即定义如何绘制未闭合线的两个端点
  pen.setJoinStyle(Qt::BevelJoin); // The join style defines how joins between two connected lines can be drawn using QPainter.
  painter.setPen(pen);

  QBrush brush;
  brush.setColor(colorBackground);
  brush.setStyle(Qt::SolidPattern);
  painter.setBrush(brush);
  rect.setRect(1,1,109,48);
  painter.drawRect(rect);

  brush.setColor(colorBorder);
  painter.setBrush(brush);

  // head of battery
  rect.setRect(110,15,10,20);
  painter.drawRect(rect);

  if(power > warnLevel)
  {
    brush.setColor(colorPower);
    pen.setColor(colorPower);
  }
  else
  {
    brush.setColor(colorWarning);
    pen.setColor(colorWarning);
  }
  painter.setBrush(brush);
  painter.setPen(pen);
  if(power > 0)
  {
    rect.setRect(5,5,power,40);
    painter.drawRect(rect);
  }

  QFontMetrics textsize(this->font());
  QString powerStr = QString::asprintf("%d%%", power);
  QRect textRect = textsize.boundingRect(powerStr);
  painter.setFont(this->font());
  pen.setColor(colorBorder);
  painter.setPen(pen);
  painter.drawText(55-textRect.width()/2,23+textRect.height()/2,powerStr);
}

void Battery::setPower(int powerIn)
{
  power = powerIn;
  repaint();
}

int Battery::getPower()
{
 return power;
}
void Battery::setWarnLevel(int warnLevelIn)
{
  warnLevel = warnLevelIn;

  repaint();
}

int Battery::getWarnLevel()
{
  return warnLevel;
}

QSize Battery::sizeHint()
{
  int H = this->height();
  int W = H * 12/5;
  QSize size(W,H);
  return size;
}
