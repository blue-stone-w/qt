#ifndef INFOLABEL_H
#define INFOLABEL_H

#include <QLabel>
#include <QObject>
#include <QString>
#include <QWidget>

class InfoLabel : public QWidget
{
  Q_OBJECT
public:
  explicit InfoLabel(QWidget *parent = nullptr);
  void setLab(QString txt="item");
  bool setInfo(QString txt="value");
  bool setParent(QWidget *parent);

signals:

public slots:
 private:
  QLabel *lab, *info; // name and info, such as "name" and "Tom"
};

#endif // INFOLABEL_H
