/********************************************************************************
** Form generated from reading UI file 'PointCloudMange.ui'
**
** Created by: Qt User Interface Compiler version 5.12.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_POINTCLOUDMANGE_H
#define UI_POINTCLOUDMANGE_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>
#include "QVTKWidget.h"

QT_BEGIN_NAMESPACE

class Ui_PointCloudMangeClass
{
public:
    QWidget *centralWidget;
    QVTKWidget *qvtkWidget;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *PointCloudMangeClass)
    {
        if (PointCloudMangeClass->objectName().isEmpty())
            PointCloudMangeClass->setObjectName(QString::fromUtf8("PointCloudMangeClass"));
        PointCloudMangeClass->resize(600, 400);
        centralWidget = new QWidget(PointCloudMangeClass);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        qvtkWidget = new QVTKWidget(centralWidget);
        qvtkWidget->setObjectName(QString::fromUtf8("qvtkWidget"));
        qvtkWidget->setGeometry(QRect(200, 60, 100, 100));
        PointCloudMangeClass->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(PointCloudMangeClass);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 600, 26));
        PointCloudMangeClass->setMenuBar(menuBar);
        mainToolBar = new QToolBar(PointCloudMangeClass);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        PointCloudMangeClass->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(PointCloudMangeClass);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        PointCloudMangeClass->setStatusBar(statusBar);

        retranslateUi(PointCloudMangeClass);

        QMetaObject::connectSlotsByName(PointCloudMangeClass);
    } // setupUi

    void retranslateUi(QMainWindow *PointCloudMangeClass)
    {
        PointCloudMangeClass->setWindowTitle(QApplication::translate("PointCloudMangeClass", "PointCloudMange", nullptr));
    } // retranslateUi

};

namespace Ui {
    class PointCloudMangeClass: public Ui_PointCloudMangeClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_POINTCLOUDMANGE_H
