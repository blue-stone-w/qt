/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.9.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
//#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QListWidget>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>
#include "SwitcherDual.h"

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *selectpath;
    QAction *savepcd;
    QAction *action;
    QAction *about;
    QWidget *centralWidget;
    QGroupBox *pcdGroup;
    QGroupBox *fileGroup;
    QListWidget *fileList;
    QGroupBox *groupBox_2;
    QGroupBox *stateGroup;
    QWidget *gridLayoutWidget;
    QGridLayout *stateLay;
    QLabel *cloudSize2;
    QLabel *time2;
    QLabel *time1;
    QLabel *indexpcd2;
    QLabel *obsnum1;
    QLabel *obsnum2;
    QLabel *cloudSize1;
    QLabel *indexpcd1;
    QGroupBox *groupBox;
    QWidget *gridLayoutWidget_3;
    QGridLayout *gridLayout;
    QLabel *displayObs1;
    SwitcherDual *switcherDual;
    QLabel *operate1;
    SwitcherDual *displayObs2;
    QLabel *displayObsCloud1;
    SwitcherDual *displayObsCloud2;
    QLabel *displayOriCloud1;
    SwitcherDual *displayOriCloud2;
    QPushButton *viewport;
    QGroupBox *playGroup;
    QPushButton *lastOne;
    QPushButton *playOne;
    QPushButton *nextOne;
    QPushButton *minusBut;
    QPushButton *addBut;
    QLineEdit *gap;
    QGroupBox *processGroup;
    QWidget *gridLayoutWidget_2;
    QGridLayout *processLay;
    QPushButton *filterpcd;
    QPushButton *mergepcd;
    QPushButton *pushButton_3;
    QPushButton *parseData;
    QLabel *filterSize1;
    QGroupBox *outoutGroup;
    QTextEdit *output;
    QMenuBar *menuBar;
    QMenu *menu;
    QMenu *help;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->setWindowModality(Qt::WindowModal);
        MainWindow->resize(1650, 878);
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(MainWindow->sizePolicy().hasHeightForWidth());
        MainWindow->setSizePolicy(sizePolicy);
        selectpath = new QAction(MainWindow);
        selectpath->setObjectName(QStringLiteral("selectpath"));
        savepcd = new QAction(MainWindow);
        savepcd->setObjectName(QStringLiteral("savepcd"));
        action = new QAction(MainWindow);
        action->setObjectName(QStringLiteral("action"));
        about = new QAction(MainWindow);
        about->setObjectName(QStringLiteral("about"));
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        pcdGroup = new QGroupBox(centralWidget);
        pcdGroup->setObjectName(QStringLiteral("pcdGroup"));
        pcdGroup->setGeometry(QRect(450, 0, 1000, 800));
        pcdGroup->setStyleSheet(QStringLiteral("background-color:transparent;"));
        fileGroup = new QGroupBox(centralWidget);
        fileGroup->setObjectName(QStringLiteral("fileGroup"));
        fileGroup->setGeometry(QRect(1460, 0, 170, 801));
        fileList = new QListWidget(fileGroup);
        fileList->setObjectName(QStringLiteral("fileList"));
        fileList->setGeometry(QRect(0, 20, 171, 781));
        fileList->setAutoFillBackground(false);
        fileList->setSelectionMode(QAbstractItemView::ExtendedSelection);
        groupBox_2 = new QGroupBox(centralWidget);
        groupBox_2->setObjectName(QStringLiteral("groupBox_2"));
        groupBox_2->setGeometry(QRect(10, 0, 431, 801));
        stateGroup = new QGroupBox(groupBox_2);
        stateGroup->setObjectName(QStringLiteral("stateGroup"));
        stateGroup->setGeometry(QRect(10, 30, 409, 91));
        gridLayoutWidget = new QWidget(stateGroup);
        gridLayoutWidget->setObjectName(QStringLiteral("gridLayoutWidget"));
        gridLayoutWidget->setGeometry(QRect(0, 30, 411, 56));
        stateLay = new QGridLayout(gridLayoutWidget);
        stateLay->setSpacing(6);
        stateLay->setContentsMargins(11, 11, 11, 11);
        stateLay->setObjectName(QStringLiteral("stateLay"));
        stateLay->setSizeConstraint(QLayout::SetDefaultConstraint);
        stateLay->setContentsMargins(5, 3, 5, 0);
        cloudSize2 = new QLabel(gridLayoutWidget);
        cloudSize2->setObjectName(QStringLiteral("cloudSize2"));
        QSizePolicy sizePolicy1(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(cloudSize2->sizePolicy().hasHeightForWidth());
        cloudSize2->setSizePolicy(sizePolicy1);
        cloudSize2->setStyleSheet(QLatin1String("border: 1px solid;\n"
"border-color: rgb(52, 101, 164);"));
        cloudSize2->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        stateLay->addWidget(cloudSize2, 0, 3, 1, 1);

        time2 = new QLabel(gridLayoutWidget);
        time2->setObjectName(QStringLiteral("time2"));
        time2->setStyleSheet(QLatin1String("border: 1px solid;\n"
"border-color: rgb(52, 101, 164);"));

        stateLay->addWidget(time2, 0, 1, 1, 1);

        time1 = new QLabel(gridLayoutWidget);
        time1->setObjectName(QStringLiteral("time1"));
        QSizePolicy sizePolicy2(QSizePolicy::Fixed, QSizePolicy::Preferred);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(time1->sizePolicy().hasHeightForWidth());
        time1->setSizePolicy(sizePolicy2);

        stateLay->addWidget(time1, 0, 0, 1, 1);

        indexpcd2 = new QLabel(gridLayoutWidget);
        indexpcd2->setObjectName(QStringLiteral("indexpcd2"));
        sizePolicy1.setHeightForWidth(indexpcd2->sizePolicy().hasHeightForWidth());
        indexpcd2->setSizePolicy(sizePolicy1);
        indexpcd2->setStyleSheet(QLatin1String("border: 1px solid;\n"
"border-color: rgb(52, 101, 164);"));
        indexpcd2->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        stateLay->addWidget(indexpcd2, 1, 1, 1, 1);

        obsnum1 = new QLabel(gridLayoutWidget);
        obsnum1->setObjectName(QStringLiteral("obsnum1"));

        stateLay->addWidget(obsnum1, 1, 2, 1, 1);

        obsnum2 = new QLabel(gridLayoutWidget);
        obsnum2->setObjectName(QStringLiteral("obsnum2"));
        QSizePolicy sizePolicy3(QSizePolicy::Preferred, QSizePolicy::Fixed);
        sizePolicy3.setHorizontalStretch(0);
        sizePolicy3.setVerticalStretch(0);
        sizePolicy3.setHeightForWidth(obsnum2->sizePolicy().hasHeightForWidth());
        obsnum2->setSizePolicy(sizePolicy3);
        obsnum2->setStyleSheet(QLatin1String("border: 1px solid;\n"
"border-color: rgb(52, 101, 164);"));
        obsnum2->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        stateLay->addWidget(obsnum2, 1, 3, 1, 1);

        cloudSize1 = new QLabel(gridLayoutWidget);
        cloudSize1->setObjectName(QStringLiteral("cloudSize1"));
        sizePolicy2.setHeightForWidth(cloudSize1->sizePolicy().hasHeightForWidth());
        cloudSize1->setSizePolicy(sizePolicy2);

        stateLay->addWidget(cloudSize1, 0, 2, 1, 1);

        indexpcd1 = new QLabel(gridLayoutWidget);
        indexpcd1->setObjectName(QStringLiteral("indexpcd1"));
        sizePolicy2.setHeightForWidth(indexpcd1->sizePolicy().hasHeightForWidth());
        indexpcd1->setSizePolicy(sizePolicy2);

        stateLay->addWidget(indexpcd1, 1, 0, 1, 1);

        groupBox = new QGroupBox(groupBox_2);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        groupBox->setGeometry(QRect(10, 150, 411, 101));
        gridLayoutWidget_3 = new QWidget(groupBox);
        gridLayoutWidget_3->setObjectName(QStringLiteral("gridLayoutWidget_3"));
        gridLayoutWidget_3->setGeometry(QRect(10, 30, 391, 51));
        gridLayout = new QGridLayout(gridLayoutWidget_3);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        gridLayout->setContentsMargins(0, 0, 0, 0);
        displayObs1 = new QLabel(gridLayoutWidget_3);
        displayObs1->setObjectName(QStringLiteral("displayObs1"));
        sizePolicy2.setHeightForWidth(displayObs1->sizePolicy().hasHeightForWidth());
        displayObs1->setSizePolicy(sizePolicy2);

        gridLayout->addWidget(displayObs1, 0, 2, 1, 1);

        switcherDual = new SwitcherDual(gridLayoutWidget_3);
        switcherDual->setObjectName(QStringLiteral("switcherDual"));

        gridLayout->addWidget(switcherDual, 0, 1, 1, 1);

        operate1 = new QLabel(gridLayoutWidget_3);
        operate1->setObjectName(QStringLiteral("operate1"));
        sizePolicy2.setHeightForWidth(operate1->sizePolicy().hasHeightForWidth());
        operate1->setSizePolicy(sizePolicy2);

        gridLayout->addWidget(operate1, 0, 0, 1, 1);

        displayObs2 = new SwitcherDual(gridLayoutWidget_3);
        displayObs2->setObjectName(QStringLiteral("displayObs2"));

        gridLayout->addWidget(displayObs2, 0, 3, 1, 1);

        displayObsCloud1 = new QLabel(gridLayoutWidget_3);
        displayObsCloud1->setObjectName(QStringLiteral("displayObsCloud1"));
        sizePolicy2.setHeightForWidth(displayObsCloud1->sizePolicy().hasHeightForWidth());
        displayObsCloud1->setSizePolicy(sizePolicy2);

        gridLayout->addWidget(displayObsCloud1, 0, 4, 1, 1);

        displayObsCloud2 = new SwitcherDual(gridLayoutWidget_3);
        displayObsCloud2->setObjectName(QStringLiteral("displayObsCloud2"));

        gridLayout->addWidget(displayObsCloud2, 0, 5, 1, 1);

        displayOriCloud1 = new QLabel(gridLayoutWidget_3);
        displayOriCloud1->setObjectName(QStringLiteral("displayOriCloud1"));
        sizePolicy2.setHeightForWidth(displayOriCloud1->sizePolicy().hasHeightForWidth());
        displayOriCloud1->setSizePolicy(sizePolicy2);

        gridLayout->addWidget(displayOriCloud1, 1, 0, 1, 1);

        displayOriCloud2 = new SwitcherDual(gridLayoutWidget_3);
        displayOriCloud2->setObjectName(QStringLiteral("displayOriCloud2"));

        gridLayout->addWidget(displayOriCloud2, 1, 1, 1, 1);

        viewport = new QPushButton(gridLayoutWidget_3);
        viewport->setObjectName(QStringLiteral("viewport"));

        gridLayout->addWidget(viewport, 1, 2, 1, 1);

        playGroup = new QGroupBox(groupBox_2);
        playGroup->setObjectName(QStringLiteral("playGroup"));
        playGroup->setGeometry(QRect(10, 270, 411, 91));
        lastOne = new QPushButton(playGroup);
        lastOne->setObjectName(QStringLiteral("lastOne"));
        lastOne->setGeometry(QRect(30, 30, 50, 50));
        lastOne->setStyleSheet(QStringLiteral("border: 1px solid;"));
        QIcon icon;
        icon.addFile(QStringLiteral("source/last.png"), QSize(), QIcon::Normal, QIcon::Off);
        lastOne->setIcon(icon);
        lastOne->setIconSize(QSize(40, 40));
        playOne = new QPushButton(playGroup);
        playOne->setObjectName(QStringLiteral("playOne"));
        playOne->setGeometry(QRect(90, 30, 50, 50));
        playOne->setStyleSheet(QStringLiteral("border: 1px solid;"));
        QIcon icon1;
        icon1.addFile(QStringLiteral("source/pause.png"), QSize(), QIcon::Normal, QIcon::Off);
        playOne->setIcon(icon1);
        playOne->setIconSize(QSize(40, 40));
        nextOne = new QPushButton(playGroup);
        nextOne->setObjectName(QStringLiteral("nextOne"));
        nextOne->setGeometry(QRect(150, 30, 50, 50));
        nextOne->setStyleSheet(QStringLiteral("border: 1px solid;"));
        QIcon icon2;
        icon2.addFile(QStringLiteral("source/next.png"), QSize(), QIcon::Normal, QIcon::Off);
        nextOne->setIcon(icon2);
        nextOne->setIconSize(QSize(40, 40));
        minusBut = new QPushButton(playGroup);
        minusBut->setObjectName(QStringLiteral("minusBut"));
        minusBut->setGeometry(QRect(220, 40, 32, 32));
        minusBut->setStyleSheet(QStringLiteral("min-width: 30px; min-height: 30px; max-width: 30px; max-height: 30px; border-radius: 15px;border: 1px solid;"));
        QIcon icon3;
        icon3.addFile(QStringLiteral("source/minus.png"), QSize(), QIcon::Normal, QIcon::Off);
        minusBut->setIcon(icon3);
        minusBut->setIconSize(QSize(32, 32));
        addBut = new QPushButton(playGroup);
        addBut->setObjectName(QStringLiteral("addBut"));
        addBut->setGeometry(QRect(350, 40, 32, 32));
        addBut->setStyleSheet(QStringLiteral("min-width: 30px; min-height: 30px; max-width: 30px; max-height: 30px; border-radius: 15px;border: 1px solid;"));
        QIcon icon4;
        icon4.addFile(QStringLiteral("source/add.jpeg"), QSize(), QIcon::Normal, QIcon::Off);
        addBut->setIcon(icon4);
        addBut->setIconSize(QSize(18, 18));
        gap = new QLineEdit(playGroup);
        gap->setObjectName(QStringLiteral("gap"));
        gap->setGeometry(QRect(260, 30, 81, 51));
        gap->setStyleSheet(QStringLiteral("border: 1px solid;"));
        gap->setAlignment(Qt::AlignCenter);
        processGroup = new QGroupBox(groupBox_2);
        processGroup->setObjectName(QStringLiteral("processGroup"));
        processGroup->setGeometry(QRect(10, 390, 411, 111));
        gridLayoutWidget_2 = new QWidget(processGroup);
        gridLayoutWidget_2->setObjectName(QStringLiteral("gridLayoutWidget_2"));
        gridLayoutWidget_2->setGeometry(QRect(10, 30, 391, 70));
        processLay = new QGridLayout(gridLayoutWidget_2);
        processLay->setSpacing(6);
        processLay->setContentsMargins(11, 11, 11, 11);
        processLay->setObjectName(QStringLiteral("processLay"));
        processLay->setContentsMargins(0, 0, 0, 0);
        filterpcd = new QPushButton(gridLayoutWidget_2);
        filterpcd->setObjectName(QStringLiteral("filterpcd"));
        QSizePolicy sizePolicy4(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy4.setHorizontalStretch(0);
        sizePolicy4.setVerticalStretch(0);
        sizePolicy4.setHeightForWidth(filterpcd->sizePolicy().hasHeightForWidth());
        filterpcd->setSizePolicy(sizePolicy4);

        processLay->addWidget(filterpcd, 0, 1, 1, 1);

        mergepcd = new QPushButton(gridLayoutWidget_2);
        mergepcd->setObjectName(QStringLiteral("mergepcd"));
        sizePolicy4.setHeightForWidth(mergepcd->sizePolicy().hasHeightForWidth());
        mergepcd->setSizePolicy(sizePolicy4);

        processLay->addWidget(mergepcd, 0, 0, 1, 1);

        pushButton_3 = new QPushButton(gridLayoutWidget_2);
        pushButton_3->setObjectName(QStringLiteral("pushButton_3"));

        processLay->addWidget(pushButton_3, 1, 0, 1, 1);

        parseData = new QPushButton(gridLayoutWidget_2);
        parseData->setObjectName(QStringLiteral("parseData"));

        processLay->addWidget(parseData, 1, 1, 1, 1);

        filterSize1 = new QLabel(gridLayoutWidget_2);
        filterSize1->setObjectName(QStringLiteral("filterSize1"));
        sizePolicy2.setHeightForWidth(filterSize1->sizePolicy().hasHeightForWidth());
        filterSize1->setSizePolicy(sizePolicy2);

        processLay->addWidget(filterSize1, 0, 2, 1, 1);

        outoutGroup = new QGroupBox(groupBox_2);
        outoutGroup->setObjectName(QStringLiteral("outoutGroup"));
        outoutGroup->setGeometry(QRect(10, 510, 411, 281));
        output = new QTextEdit(outoutGroup);
        output->setObjectName(QStringLiteral("output"));
        output->setGeometry(QRect(0, 20, 411, 261));
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1650, 28));
        menu = new QMenu(menuBar);
        menu->setObjectName(QStringLiteral("menu"));
        help = new QMenu(menuBar);
        help->setObjectName(QStringLiteral("help"));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        MainWindow->setStatusBar(statusBar);

        menuBar->addAction(menu->menuAction());
        menuBar->addAction(help->menuAction());
        menu->addAction(selectpath);
        menu->addAction(savepcd);
        menu->addSeparator();
        menu->addAction(action);
        help->addAction(about);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "\347\202\271\344\272\221\345\210\206\346\236\220", Q_NULLPTR));
        selectpath->setText(QApplication::translate("MainWindow", "\351\200\211\346\213\251\346\226\207\344\273\266\345\244\271", Q_NULLPTR));
        selectpath->setIconText(QApplication::translate("MainWindow", "\351\200\211\346\226\207\344\273\266\345\244\271", Q_NULLPTR));
        savepcd->setText(QApplication::translate("MainWindow", "\344\277\235\345\255\230\347\202\271\344\272\221", Q_NULLPTR));
        action->setText(QApplication::translate("MainWindow", "\350\277\234\347\250\213\346\225\260\346\215\256\350\216\267\345\217\226", Q_NULLPTR));
        about->setText(QApplication::translate("MainWindow", "\345\205\263\344\272\216", Q_NULLPTR));
        pcdGroup->setTitle(QApplication::translate("MainWindow", "\346\230\276\347\244\272\347\273\204", Q_NULLPTR));
        fileGroup->setTitle(QApplication::translate("MainWindow", "\346\227\266\351\227\264\347\273\204", Q_NULLPTR));
        groupBox_2->setTitle(QApplication::translate("MainWindow", "\350\260\203\345\272\246\347\273\204", Q_NULLPTR));
        stateGroup->setTitle(QApplication::translate("MainWindow", "\345\237\272\346\234\254\344\277\241\346\201\257", Q_NULLPTR));
        cloudSize2->setText(QApplication::translate("MainWindow", "0,0", Q_NULLPTR));
        time2->setText(QString());
        time1->setText(QApplication::translate("MainWindow", "\347\202\271\344\272\221\346\227\266\351\227\264: ", Q_NULLPTR));
        indexpcd2->setText(QApplication::translate("MainWindow", "0 / 0", Q_NULLPTR));
        obsnum1->setText(QApplication::translate("MainWindow", "\351\232\234\347\242\215\346\225\260\351\207\217: ", Q_NULLPTR));
        obsnum2->setText(QApplication::translate("MainWindow", "0", Q_NULLPTR));
        cloudSize1->setText(QApplication::translate("MainWindow", "\347\202\271\344\272\221\346\225\260\351\207\217: ", Q_NULLPTR));
        indexpcd1->setText(QApplication::translate("MainWindow", "\347\202\271\344\272\221\345\272\217\345\217\267: ", Q_NULLPTR));
        groupBox->setTitle(QApplication::translate("MainWindow", "\346\230\276\347\244\272\346\216\247\345\210\266", Q_NULLPTR));
        displayObs1->setText(QApplication::translate("MainWindow", "\346\230\276\347\244\272\347\211\251\344\275\223:", Q_NULLPTR));
        operate1->setText(QApplication::translate("MainWindow", "\347\202\271\344\272\221\345\244\204\347\220\206: ", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        displayObs2->setToolTip(QApplication::translate("MainWindow", "\351\200\211\346\213\251\346\230\257\345\220\246\346\230\276\347\244\272\351\232\234\347\242\215\347\211\251\345\214\205\345\233\264\346\241\206", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        displayObsCloud1->setText(QApplication::translate("MainWindow", "\347\211\251\344\275\223\347\202\271\344\272\221:", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        displayObsCloud2->setToolTip(QApplication::translate("MainWindow", "\351\200\211\346\213\251\346\230\257\345\220\246\346\230\276\347\244\272\351\232\234\347\242\215\347\211\251\347\202\271\344\272\221", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        displayOriCloud1->setText(QApplication::translate("MainWindow", "\345\216\237\345\247\213\347\202\271\344\272\221:", Q_NULLPTR));
#ifndef QT_NO_TOOLTIP
        displayOriCloud2->setToolTip(QApplication::translate("MainWindow", "\351\200\211\346\213\251\346\230\257\345\220\246\346\230\276\347\244\272\345\216\237\345\247\213\347\202\271\344\272\221", Q_NULLPTR));
#endif // QT_NO_TOOLTIP
        viewport->setText(QApplication::translate("MainWindow", "\351\207\215\347\275\256\350\247\206\350\247\222", Q_NULLPTR));
        playGroup->setTitle(QApplication::translate("MainWindow", "\346\222\255\346\224\276\346\216\247\345\210\266", Q_NULLPTR));
        lastOne->setText(QString());
        playOne->setText(QString());
        nextOne->setText(QString());
        minusBut->setText(QString());
        addBut->setText(QString());
        gap->setText(QApplication::translate("MainWindow", "0.5", Q_NULLPTR));
        processGroup->setTitle(QApplication::translate("MainWindow", "\347\202\271\344\272\221\345\244\204\347\220\206", Q_NULLPTR));
        filterpcd->setText(QApplication::translate("MainWindow", "\347\202\271\344\272\221\346\273\244\346\263\242", Q_NULLPTR));
        mergepcd->setText(QApplication::translate("MainWindow", "\345\220\210\345\271\266\347\202\271\344\272\221", Q_NULLPTR));
        pushButton_3->setText(QApplication::translate("MainWindow", "record #1", Q_NULLPTR));
        parseData->setText(QApplication::translate("MainWindow", "\350\277\234\347\250\213\350\277\236\346\216\245", Q_NULLPTR));
        filterSize1->setText(QApplication::translate("MainWindow", "\346\273\244\346\263\242\345\260\272\345\257\270: ", Q_NULLPTR));
        outoutGroup->setTitle(QApplication::translate("MainWindow", "\350\276\223\345\207\272", Q_NULLPTR));
        menu->setTitle(QApplication::translate("MainWindow", "\346\226\207\344\273\266", Q_NULLPTR));
        help->setTitle(QApplication::translate("MainWindow", "\345\270\256\345\212\251", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
