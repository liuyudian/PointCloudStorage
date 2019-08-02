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
#include <QtWidgets/QApplication>
#include <QtWidgets/QMainWindow>
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
    QWidget *centralWidget;
    QVTKWidget *qvtkWidget;
    QPushButton *pushButton_2;
    QPushButton *pushButton;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *PointCloudManageClass)
    {
        if (PointCloudManageClass->objectName().isEmpty())
            PointCloudManageClass->setObjectName(QString::fromUtf8("PointCloudManageClass"));
        PointCloudManageClass->resize(600, 400);
        centralWidget = new QWidget(PointCloudManageClass);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        qvtkWidget = new QVTKWidget(centralWidget);
        qvtkWidget->setObjectName(QString::fromUtf8("qvtkWidget"));
        qvtkWidget->setGeometry(QRect(110, 9, 451, 311));
        pushButton_2 = new QPushButton(centralWidget);
        pushButton_2->setObjectName(QString::fromUtf8("pushButton_2"));
        pushButton_2->setGeometry(QRect(0, 70, 93, 28));
        pushButton = new QPushButton(centralWidget);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));
        pushButton->setGeometry(QRect(0, 10, 93, 28));
        PointCloudManageClass->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(PointCloudManageClass);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 600, 26));
        PointCloudManageClass->setMenuBar(menuBar);
        mainToolBar = new QToolBar(PointCloudManageClass);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        PointCloudManageClass->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(PointCloudManageClass);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        PointCloudManageClass->setStatusBar(statusBar);

        retranslateUi(PointCloudManageClass);

        QMetaObject::connectSlotsByName(PointCloudManageClass);
    } // setupUi

    void retranslateUi(QMainWindow *PointCloudManageClass)
    {
        PointCloudManageClass->setWindowTitle(QApplication::translate("PointCloudManageClass", "PointCloudManage", nullptr));
        pushButton_2->setText(QApplication::translate("PointCloudManageClass", "\345\217\246\345\255\230\344\270\272", nullptr));
        pushButton->setText(QApplication::translate("PointCloudManageClass", "\346\211\223\345\274\200\346\226\207\344\273\266", nullptr));
    } // retranslateUi

};

namespace Ui {
    class PointCloudManageClass: public Ui_PointCloudManageClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_POINTCLOUDMANAGE_H
