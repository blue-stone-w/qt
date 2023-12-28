#include "InfoLabel.h"

InfoLabel::InfoLabel(QWidget *parent) : QWidget(parent)
{

}

bool InfoLabel::setInfo(QString txt)
{
  info->setText(txt);
  return true;
}
bool InfoLabel::setParent(QWidget *parent)
{
  this->setParent(parent);
  return true;
}
void InfoLabel::setLab(QString txt)
{
  lab->setText(txt);
  return;
}
