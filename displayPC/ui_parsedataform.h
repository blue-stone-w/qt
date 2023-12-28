/********************************************************************************
** Form generated from reading UI file 'parsedataform.ui'
**
** Created by: Qt User Interface Compiler version 5.9.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PARSEDATAFORM_H
#define UI_PARSEDATAFORM_H

#include <QtCore/QVariant>
#include <QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QProgressBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_ParseDataForm
{
public:
    QComboBox *comboBox;
    QLabel *label;
    QLabel *time1;
    QLineEdit *time2;
    QGroupBox *groupBox;
    QWidget *gridLayoutWidget;
    QGridLayout *gridLayout;
    QLineEdit *ip2;
    QLabel *lf1;
    QLabel *ip1;
    QPushButton *saveButton;
    QLineEdit *lf2;
    QLabel *bf1;
    QLineEdit *lb2;
    QLabel *savepath1;
    QLineEdit *savepath2;
    QGroupBox *groupBox_2;
    QWidget *gridLayoutWidget_2;
    QGridLayout *gridLayout_2;
    QPushButton *start;
    QPushButton *end;
    QPushButton *parse;
    QPushButton *transmit;
    QPushButton *allAuto;
    QCheckBox *checkBox;
    QProgressBar *progressBar;

    void setupUi(QWidget *ParseDataForm)
    {
        if (ParseDataForm->objectName().isEmpty())
            ParseDataForm->setObjectName(QStringLiteral("ParseDataForm"));
        ParseDataForm->resize(359, 650);
        comboBox = new QComboBox(ParseDataForm);
        comboBox->setObjectName(QStringLiteral("comboBox"));
        comboBox->setGeometry(QRect(120, 20, 141, 25));
        label = new QLabel(ParseDataForm);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(30, 20, 67, 17));
        time1 = new QLabel(ParseDataForm);
        time1->setObjectName(QStringLiteral("time1"));
        time1->setGeometry(QRect(30, 60, 67, 17));
        time2 = new QLineEdit(ParseDataForm);
        time2->setObjectName(QStringLiteral("time2"));
        time2->setGeometry(QRect(120, 60, 191, 25));
        groupBox = new QGroupBox(ParseDataForm);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        groupBox->setGeometry(QRect(30, 100, 291, 281));
        gridLayoutWidget = new QWidget(groupBox);
        gridLayoutWidget->setObjectName(QStringLiteral("gridLayoutWidget"));
        gridLayoutWidget->setGeometry(QRect(10, 30, 271, 201));
        gridLayout = new QGridLayout(gridLayoutWidget);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        gridLayout->setContentsMargins(0, 0, 0, 0);
        ip2 = new QLineEdit(gridLayoutWidget);
        ip2->setObjectName(QStringLiteral("ip2"));

        gridLayout->addWidget(ip2, 0, 1, 1, 1);

        lf1 = new QLabel(gridLayoutWidget);
        lf1->setObjectName(QStringLiteral("lf1"));

        gridLayout->addWidget(lf1, 1, 0, 1, 1);

        ip1 = new QLabel(gridLayoutWidget);
        ip1->setObjectName(QStringLiteral("ip1"));

        gridLayout->addWidget(ip1, 0, 0, 1, 1);

        saveButton = new QPushButton(gridLayoutWidget);
        saveButton->setObjectName(QStringLiteral("saveButton"));

        gridLayout->addWidget(saveButton, 4, 0, 1, 2);

        lf2 = new QLineEdit(gridLayoutWidget);
        lf2->setObjectName(QStringLiteral("lf2"));

        gridLayout->addWidget(lf2, 1, 1, 1, 1);

        bf1 = new QLabel(gridLayoutWidget);
        bf1->setObjectName(QStringLiteral("bf1"));

        gridLayout->addWidget(bf1, 2, 0, 1, 1);

        lb2 = new QLineEdit(gridLayoutWidget);
        lb2->setObjectName(QStringLiteral("lb2"));

        gridLayout->addWidget(lb2, 2, 1, 1, 1);

        savepath1 = new QLabel(gridLayoutWidget);
        savepath1->setObjectName(QStringLiteral("savepath1"));

        gridLayout->addWidget(savepath1, 3, 0, 1, 1);

        savepath2 = new QLineEdit(gridLayoutWidget);
        savepath2->setObjectName(QStringLiteral("savepath2"));

        gridLayout->addWidget(savepath2, 3, 1, 1, 1);

        groupBox_2 = new QGroupBox(ParseDataForm);
        groupBox_2->setObjectName(QStringLiteral("groupBox_2"));
        groupBox_2->setGeometry(QRect(30, 390, 291, 161));
        gridLayoutWidget_2 = new QWidget(groupBox_2);
        gridLayoutWidget_2->setObjectName(QStringLiteral("gridLayoutWidget_2"));
        gridLayoutWidget_2->setGeometry(QRect(10, 30, 271, 121));
        gridLayout_2 = new QGridLayout(gridLayoutWidget_2);
        gridLayout_2->setObjectName(QStringLiteral("gridLayout_2"));
        gridLayout_2->setContentsMargins(0, 0, 0, 0);
        start = new QPushButton(gridLayoutWidget_2);
        start->setObjectName(QStringLiteral("start"));

        gridLayout_2->addWidget(start, 0, 0, 1, 1);

        end = new QPushButton(gridLayoutWidget_2);
        end->setObjectName(QStringLiteral("end"));

        gridLayout_2->addWidget(end, 0, 1, 1, 1);

        parse = new QPushButton(gridLayoutWidget_2);
        parse->setObjectName(QStringLiteral("parse"));

        gridLayout_2->addWidget(parse, 1, 1, 1, 1);

        transmit = new QPushButton(gridLayoutWidget_2);
        transmit->setObjectName(QStringLiteral("transmit"));

        gridLayout_2->addWidget(transmit, 1, 0, 1, 1);

        allAuto = new QPushButton(gridLayoutWidget_2);
        allAuto->setObjectName(QStringLiteral("allAuto"));

        gridLayout_2->addWidget(allAuto, 2, 0, 1, 2);

        checkBox = new QCheckBox(ParseDataForm);
        checkBox->setObjectName(QStringLiteral("checkBox"));
        checkBox->setGeometry(QRect(270, 20, 92, 23));
        progressBar = new QProgressBar(ParseDataForm);
        progressBar->setObjectName(QStringLiteral("progressBar"));
        progressBar->setGeometry(QRect(30, 580, 291, 23));
        progressBar->setValue(24);

        retranslateUi(ParseDataForm);

        QMetaObject::connectSlotsByName(ParseDataForm);
    } // setupUi

    void retranslateUi(QWidget *ParseDataForm)
    {
        ParseDataForm->setWindowTitle(QApplication::translate("ParseDataForm", "Form", Q_NULLPTR));
        label->setText(QApplication::translate("ParseDataForm", "\350\275\246\350\276\206\351\200\211\346\213\251", Q_NULLPTR));
        time1->setText(QApplication::translate("ParseDataForm", "\345\275\225\345\210\266\346\227\266\351\227\264", Q_NULLPTR));
        groupBox->setTitle(QApplication::translate("ParseDataForm", "\350\275\246\350\276\206\345\217\202\346\225\260", Q_NULLPTR));
        lf1->setText(QApplication::translate("ParseDataForm", "\345\211\215\351\233\267\350\276\276", Q_NULLPTR));
        ip1->setText(QApplication::translate("ParseDataForm", "IP", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        saveButton->setToolTip(QApplication::translate("ParseDataForm", "<html><head/><body><p>\344\277\235\345\255\230\347\232\204\345\217\202\346\225\260\344\273\205\345\234\250\350\257\245\347\252\227\345\217\243\346\234\211\346\225\210\357\274\214\344\270\215\344\274\232\346\233\264\346\224\271\351\205\215\347\275\256\346\226\207\344\273\266</p></body></html>", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        saveButton->setText(QApplication::translate("ParseDataForm", "\344\277\235\345\255\230\345\217\202\346\225\260", Q_NULLPTR));
        bf1->setText(QApplication::translate("ParseDataForm", "\345\220\216\351\233\267\350\276\276", Q_NULLPTR));
        savepath1->setText(QApplication::translate("ParseDataForm", "\344\277\235\345\255\230\350\267\257\345\276\204", Q_NULLPTR));
        groupBox_2->setTitle(QApplication::translate("ParseDataForm", "\346\223\215\344\275\234\351\200\211\351\241\271", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        start->setToolTip(QApplication::translate("ParseDataForm", "<html><head/><body><p>\347\202\271\345\207\273\345\274\200\345\247\213\345\275\225\345\210\266\345\214\205</p></body></html>", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        start->setText(QApplication::translate("ParseDataForm", "\345\274\200\345\247\213\345\275\225\345\210\266", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        end->setToolTip(QApplication::translate("ParseDataForm", "<html><head/><body><p>\347\202\271\345\207\273\347\273\223\346\235\237\345\275\225\345\210\266\345\214\205</p></body></html>", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        end->setText(QApplication::translate("ParseDataForm", "\347\273\223\346\235\237\345\275\225\345\210\266", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        parse->setToolTip(QApplication::translate("ParseDataForm", "<html><head/><body><p>\346\212\212\346\225\260\346\215\256\350\247\243\346\236\220\344\270\272\350\260\203\350\257\225\347\232\204\346\226\207\344\273\266\346\240\274\345\274\217</p></body></html>", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        parse->setText(QApplication::translate("ParseDataForm", "\350\247\243\346\236\220\346\225\260\346\215\256", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        transmit->setToolTip(QApplication::translate("ParseDataForm", "<html><head/><body><p>\346\212\212\346\225\260\346\215\256\344\274\240\351\200\222\350\207\263\346\234\254\345\234\260</p></body></html>", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        transmit->setText(QApplication::translate("ParseDataForm", "\346\213\211\345\217\226\346\225\260\346\215\256", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        allAuto->setToolTip(QApplication::translate("ParseDataForm", "<html><head/><body><p>\346\214\211\347\205\247\350\256\276\345\256\232\347\232\204\350\247\204\345\210\231\350\207\252\345\212\250\345\256\214\346\210\220\345\275\225\345\210\266\346\213\211\345\217\226\350\247\243\346\236\220; \347\224\250\346\227\266\351\227\264\345\221\275\345\220\215\346\226\207\344\273\266\345\244\271</p></body></html>", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        allAuto->setText(QApplication::translate("ParseDataForm", "\344\270\200\351\224\256\345\256\214\346\210\220", Q_NULLPTR));
        checkBox->setText(QApplication::translate("ParseDataForm", "\347\275\221\347\273\234\350\277\236\346\216\245", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class ParseDataForm: public Ui_ParseDataForm {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PARSEDATAFORM_H
