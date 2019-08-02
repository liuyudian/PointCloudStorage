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
        qvtkWidget->setGeometry(QRect(260, 90, 100, 100));
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
    } // retranslateUi

};

namespace Ui {
    class PointCloudManageClass: public Ui_PointCloudManageClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_POINTCLOUDMANAGE_H
