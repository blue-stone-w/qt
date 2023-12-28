#include "Bar.h"

Bar::Bar() : QWidget() {}
Bar::Bar(QWidget *parent) : QWidget(parent)
{
  win = parent;
  background = new QLabel(win);
  background->setStyleSheet("QLabel{background-color:transparent;border:1px solid white}");
  colorbar = new QLabel(parent);
  setGeometry(250,550,500,50);
  resize(500,50);
  setCurrValue();
}

Bar::~Bar()
{
  delete background;
}

void Bar::resize(int w, int h)
{
  width = w;
  height = h;
  background->resize(width, height);
}

void Bar::setGeometry(int x, int y, int w, int h)
{
  px = x, py = y, width = w, height = h;
  background->setGeometry(px,py,width,height);
    colorbar->setGeometry(px+1,py+1,width-2,height-2);
    colorbar->setAutoFillBackground(true);
}

void Bar::setCurrValue(int valueIn)
{
  if (valueIn >= 0 && valueIn < 100)
  {
    currPentcent = valueIn;
  }
  else if (valueIn < 0)
  {
    currPentcent = 0;
  }
  else if (valueIn > 100)
  {
    currPentcent = 100;
  }

  colorbar->resize(currPentcent*width/100, height-2);

  QPalette pa;
  if (currPentcent > 40)
  {
    pa.setColor(QPalette::Window, QColor(255*(100-currPentcent)/(100-40),255,0));
  }
  else if (currPentcent > 15)
  {
    pa.setColor(QPalette::Window, QColor(255,255*(currPentcent-15)/(40-15),0));
  }
  else
  {
    pa.setColor(QPalette::Window, QColor(255,0,0));
  }
  colorbar->setPalette(pa);
}
