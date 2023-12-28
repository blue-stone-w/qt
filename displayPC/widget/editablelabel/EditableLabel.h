#ifndef EDITABLELABEL_H
#define EDITABLELABEL_H

#include <QEvent>
#include <QDebug>
#include <QKeyEvent>
#include <QLabel>
#include <QLineEdit>
#include <QObject>
#include <QStackedWidget>
#include <QString>
#include <QWidget>

#include <QMessageBox>

class EditableLabel : public QWidget
{
  Q_OBJECT
 public:
  EditableLabel(QWidget *parent = Q_NULLPTR);
  QStackedWidget *stack;
  QLabel *label;
  QLineEdit *lineEdit;
 protected:
  bool eventFilter(QObject *obj, QEvent *e);

 signals:
  void saveSignal();

 private:
  void init(void);

};

/*
 *   ll = new EditableLabel(ui->imgLab);
 *   ll->move(300,400);
*/
#endif // EDITABLELABEL_H
