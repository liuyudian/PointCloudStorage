#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_PointCloudManage.h"

#include <QMainWindow>
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

private:
	Ui::PointCloudManageClass ui;
	pcl::visualization::PCLVisualizer::Ptr viewer;
	PointCloudT::Ptr cloud;
	unsigned int red;
	unsigned int green;
	unsigned int blue;

public slots:
	void OpenFile();

};
