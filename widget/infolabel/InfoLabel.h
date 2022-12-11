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
  bool setInfo(QString txt);
  bool setParent(QWidget *parent);

signals:

public slots:
 private:
  QLabel *lab, *info; // name and info, such as "name" and "Tom"
  void setLab(QString txt);
};

#endif // INFOLABEL_H
