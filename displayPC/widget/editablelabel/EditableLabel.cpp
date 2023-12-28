#include "EditableLabel.h"

EditableLabel::EditableLabel(QWidget *parent) : QWidget(parent)
{
  init();
}

bool EditableLabel::eventFilter(QObject *obj, QEvent *e)
{
  if(obj == label)
  {
    if(QEvent::FocusIn == e->type())
    {
      stack->setCurrentWidget(lineEdit);
    }
  }
  else if(obj == lineEdit)
  {
    if(QEvent::FocusOut == e->type())
    {
      stack->setCurrentWidget(label);
    }
    else if(e->type() == QEvent::KeyPress)
    {
      QKeyEvent *keyevt = static_cast<QKeyEvent*>(e);
      if( (keyevt->key() == Qt::Key_Return) || (keyevt->key() == Qt::Key_Enter) )
      {
        // varify
//        if(lineEdit->text.size() == 0)
//        {
//          QMessageBox *temp1(new QMessageBox(QMessageBox::Warning, "12","34"));
//        }
        label->setText(lineEdit->text());
        stack->setCurrentWidget(label);
        lineEdit->clearFocus();
        emit saveSignal();
      }
      else if(keyevt->key() == Qt::Key_Escape)
      {
        stack->setCurrentWidget(label);
        lineEdit->clearFocus();
      }
    }
  }
  return QWidget::eventFilter(obj,e);
}

void EditableLabel::init(void)
{
  stack = new QStackedWidget(this);

  label = new QLabel("10");
  label->setStyleSheet("QLabel{font:20px;background-color:rgb(0,250,250);}");
  label->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);

  lineEdit = new QLineEdit();
  lineEdit->setAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
  lineEdit->setStyle(label->style());

  label->setFocusPolicy(Qt::FocusPolicy::ClickFocus); // mouse double click

  stack->setGeometry(0,0,50,25);

  label->installEventFilter(this);
  lineEdit->installEventFilter(this);

  stack->addWidget(label);
  stack->addWidget(lineEdit);

  stack->setCurrentWidget(label);
}
