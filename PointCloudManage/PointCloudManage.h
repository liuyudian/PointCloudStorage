#pragma once
//QT的头文件
#include <QtWidgets/QMainWindow>
#include "ui_PointCloudManage.h"
#include <QFileInfo>//qt文件操作头文件
#include <QMainWindow>
#include <QApplication>
#include <QFileDialog>
#include <QDebug>
#include <QString>


//pcl和vtk的头文件
#include <vtkRenderWindow.h>
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include<pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
#include "vtkAutoInit.h"

VTK_MODULE_INIT(vtkRenderingOpenGL2);

VTK_MODULE_INIT(vtkInteractionStyle);
class PointCloudManage : public QMainWindow
{
	Q_OBJECT


public:
	PointCloudManage(QWidget *parent = Q_NULLPTR);

protected:
	pcl::visualization::PCLVisualizer::Ptr viewer;
	PointCloudT::Ptr cloud;

	unsigned int red;
	unsigned int green;
	unsigned int blue;


private:
	Ui::PointCloudManageClass *ui;

public slots:
	void OpenFile(); //打开文件
	void VTK_Show(QString s);//在vtk控件中显示点云

};
