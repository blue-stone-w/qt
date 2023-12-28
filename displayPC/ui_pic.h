/********************************************************************************
** Form generated from reading UI file 'pic.ui'
**
** Created by: Qt User Interface Compiler version 5.9.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PIC_H
#define UI_PIC_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Form
{
public:
    QGroupBox *imgGroup;
    QLabel *imgLab;

    void setupUi(QWidget *Form)
    {
        if (Form->objectName().isEmpty())
            Form->setObjectName(QStringLiteral("Form"));
        Form->resize(474, 332);
        imgGroup = new QGroupBox(Form);
        imgGroup->setObjectName(QStringLiteral("imgGroup"));
        imgGroup->setGeometry(QRect(10, 10, 451, 311));
        imgLab = new QLabel(imgGroup);
        imgLab->setObjectName(QStringLiteral("imgLab"));
        imgLab->setGeometry(QRect(10, 30, 341, 191));
        imgLab->setScaledContents(true);

        retranslateUi(Form);

        QMetaObject::connectSlotsByName(Form);
    } // setupUi

    void retranslateUi(QWidget *Form)
    {
        Form->setWindowTitle(QApplication::translate("Form", "Form", Q_NULLPTR));
        imgGroup->setTitle(QApplication::translate("Form", "\345\233\276\347\211\207", Q_NULLPTR));
        imgLab->setText(QString());
    } // retranslateUi

};

namespace Ui {
    class Form: public Ui_Form {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PIC_H
