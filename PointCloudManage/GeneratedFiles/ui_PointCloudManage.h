/********************************************************************************
** Form generated from reading UI file 'PointCloudManage.ui'
**
** Created by: Qt User Interface Compiler version 5.12.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_POINTCLOUDMANAGE_H
#define UI_POINTCLOUDMANAGE_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>
#include "QVTKWidget.h"

QT_BEGIN_NAMESPACE

class Ui_PointCloudManageClass
{
public:
    QAction *action11;
    QWidget *centralWidget;
    QVTKWidget *qvtkWidget;
    QPushButton *pushButton_2;
    QPushButton *pushButton;
    QPushButton *pushButton3;
    QPushButton *pushButton_4;
    QPushButton *pushButton_5;
    QMenuBar *menuBar;
    QMenu *menu11;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *PointCloudManageClass)
    {
        if (PointCloudManageClass->objectName().isEmpty())
            PointCloudManageClass->setObjectName(QString::fromUtf8("PointCloudManageClass"));
        PointCloudManageClass->resize(1002, 639);
        action11 = new QAction(PointCloudManageClass);
        action11->setObjectName(QString::fromUtf8("action11"));
        centralWidget = new QWidget(PointCloudManageClass);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        qvtkWidget = new QVTKWidget(centralWidget);
        qvtkWidget->setObjectName(QString::fromUtf8("qvtkWidget"));
        qvtkWidget->setGeometry(QRect(110, 10, 861, 571));
        pushButton_2 = new QPushButton(centralWidget);
        pushButton_2->setObjectName(QString::fromUtf8("pushButton_2"));
        pushButton_2->setGeometry(QRect(0, 70, 93, 28));
        pushButton = new QPushButton(centralWidget);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));
        pushButton->setGeometry(QRect(0, 10, 93, 28));
        pushButton3 = new QPushButton(centralWidget);
        pushButton3->setObjectName(QString::fromUtf8("pushButton3"));
        pushButton3->setGeometry(QRect(0, 140, 93, 28));
        pushButton_4 = new QPushButton(centralWidget);
        pushButton_4->setObjectName(QString::fromUtf8("pushButton_4"));
        pushButton_4->setGeometry(QRect(0, 200, 93, 28));
        pushButton_5 = new QPushButton(centralWidget);
        pushButton_5->setObjectName(QString::fromUtf8("pushButton_5"));
        pushButton_5->setGeometry(QRect(0, 290, 93, 28));
        PointCloudManageClass->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(PointCloudManageClass);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1002, 26));
        menu11 = new QMenu(menuBar);
        menu11->setObjectName(QString::fromUtf8("menu11"));
        PointCloudManageClass->setMenuBar(menuBar);
        mainToolBar = new QToolBar(PointCloudManageClass);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        PointCloudManageClass->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(PointCloudManageClass);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        PointCloudManageClass->setStatusBar(statusBar);

        menuBar->addAction(menu11->menuAction());
        menu11->addAction(action11);

        retranslateUi(PointCloudManageClass);

        QMetaObject::connectSlotsByName(PointCloudManageClass);
    } // setupUi

    void retranslateUi(QMainWindow *PointCloudManageClass)
    {
        PointCloudManageClass->setWindowTitle(QApplication::translate("PointCloudManageClass", "PointCloudManage", nullptr));
        action11->setText(QApplication::translate("PointCloudManageClass", "\346\211\223\345\274\200\346\226\207\344\273\266", nullptr));
        pushButton_2->setText(QApplication::translate("PointCloudManageClass", "\345\217\246\345\255\230\344\270\272", nullptr));
        pushButton->setText(QApplication::translate("PointCloudManageClass", "\346\211\223\345\274\200\346\226\207\344\273\266", nullptr));
        pushButton3->setText(QApplication::translate("PointCloudManageClass", "\346\250\241\345\236\213\346\230\276\347\244\272", nullptr));
        pushButton_4->setText(QApplication::translate("PointCloudManageClass", "\345\217\266\345\255\220\350\212\202\347\202\271", nullptr));
        pushButton_5->setText(QApplication::translate("PointCloudManageClass", "\344\270\211\350\247\222\347\275\221\346\240\274\345\211\226\345\210\206", nullptr));
        menu11->setTitle(QApplication::translate("PointCloudManageClass", "\346\226\207\344\273\266", nullptr));
    } // retranslateUi

};

namespace Ui {
    class PointCloudManageClass: public Ui_PointCloudManageClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_POINTCLOUDMANAGE_H
