#include "AzimuthPanel.h"

AzimuthPanel::AzimuthPanel(): QWidget()
{
  AzimuthPanel::initAzimuthPanel();
}

AzimuthPanel::AzimuthPanel(QWidget *parent): QWidget(parent)
{
  AzimuthPanel::initAzimuthPanel();
}

AzimuthPanel::~AzimuthPanel() {}

void AzimuthPanel::initAzimuthPanel()
{
  angle=40;

  //填充表针的颜色
  red=QColor(0xcc,0x33,0x33,255);  //指针颜色(第四个表示不透明度)
  mainColor=QColor(0x4E,0x6D,0x8C,255);  //指针颜色(第四个表示不透明度)

  resize(300,300);
  W = width();
  H = height();
  side = qMin( width(),  height());  //绘制的范围(宽、高中最小值)
}

void AzimuthPanel::paintEvent(QPaintEvent *noevent)
{
  Q_UNUSED(noevent);
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing,true);//必须先激活才能有这种操作
  painter.setRenderHint(QPainter::SmoothPixmapTransform,true);

  painter.translate(0.5*W,0.5*H);
  painter.scale(side / 310.0, side / 310.0);//设定画布的边界,用窗体宽高的最小值来计算时钟的大小，防止窗体拉伸导致的时钟变形以及显示不全

  drawNumberSpeed(painter);
  drawFrame(painter); // AzimuthPanel
  drawDividing(painter); //
  drawIndicator(painter); // speed pointer


    painter.end();                     // painting done


}

void AzimuthPanel::drawFrame(QPainter& painter)
{
  painter.save();
  QPen pen;
  pen.setWidth(4);//
  pen.setColor(mainColor);

  painter.setPen(pen);
  painter.drawEllipse(-152,-152,304,304);//绘边框

  painter.restore();
}

void AzimuthPanel::drawDividing(QPainter &painter)
{
  painter.save();
  QFont font=painter.font();
  int ang=0;//间隔角度
  for(int i=0;i<360;i++)
  {
    font.setPixelSize(10);
    painter.setFont(font);
    if((i%10)==0)
    {//绘制长线
      if((i%90)==0)
      {
        painter.setPen(red);
        if(i==0)
        {
          painter.drawLine(0,-115,0,-125);
          painter.drawText(-8,-113,16,10,Qt::AlignHCenter | Qt::AlignTop,"N");

        }
        else if(i==90)
        {
          painter.drawLine(0,-115,0,-125);
          painter.drawText(-8,-113,16,10,Qt::AlignHCenter | Qt::AlignTop,"E");

        }
        else if(i==180)
        {
          painter.drawLine(0,-115,0,-125);
          painter.drawText(-8,-113,16,10,Qt::AlignHCenter | Qt::AlignTop,"S");

        }
        else if(i==270)
        {
          painter.drawLine(0,-115,0,-125);
          painter.drawText(-8,-113,16,10,Qt::AlignHCenter | Qt::AlignTop,"W");

        }
      }
      painter.setPen(red);
      painter.drawLine(0,-143,0,-150);
      painter.drawText(-8,-138,16,10,Qt::AlignHCenter | Qt::AlignTop,QString::number(ang*10));
      ang++;
    }
    else
    {
      painter.setPen(mainColor);
      if((i%5)==0)
      {
        painter.drawLine(0,-145,0,-150);
      }
      else
      {
        painter.drawLine(0,-146,0,-150);
      }

    }

    painter.rotate(1);
  }
  painter.restore();
}

void AzimuthPanel::drawIndicator(QPainter &painter)
{
  painter.save();
  painter.setPen(QPen(red));  //填充时针的区域
  painter.setBrush(red);
  painter.rotate(angle);
  painter.drawConvexPolygon(hourHand, 3);
  painter.drawEllipse(-12,-12,24,24);
  painter.restore();
}

void AzimuthPanel::drawNumberSpeed(QPainter &painter)
{
  painter.save();

  QFont font=painter.font();
  font.setPixelSize(20);
  font.setFamily("YouYuan");
  painter.setFont(font);
  QPen pen;
  painter.setPen(mainColor);

  painter.drawText(0,0.1*H,100,25,Qt::AlignLeft | Qt::AlignTop,QString("%1 ").arg(angle));
  painter.restore();
}


void AzimuthPanel::setCurrentValue(float valueIn)
{
  angle = valueIn;
  update();
}

