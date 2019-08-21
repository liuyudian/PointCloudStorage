#pragma once
//自定义类文件
#include"FileOption.h"

//QT的头文件
#include <QtWidgets/QMainWindow>
#include "ui_PointCloudManage.h"
#include <QFileInfo>//qt文件操作头文件
#include <QMainWindow>
#include <QApplication>
#include <QFileDialog>
#include <QDebug>
#include <QString>
#include <QAction>


//pcl和vtk的头文件
#include <vtkRenderWindow.h>
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include<pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
//空间划分头文件
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <fstream>
#include <iostream>
#include <math.h>
#include <stdio.h>
#include <string.h>
using namespace std;


typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointXYZRGBA LeafT;
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
	void ShowModel(); //打开文件
	void VTK_Show(string s);//在vtk控件中显示点云
	void SaveAsPLY();//把最终的模型另存为ply文件
	void GetLeafShow();
	void Triangulation();//三角网格剖分
	void MeshGeneration();//自定义网格剖分
	void Gridding();//种子三角面片获取
};
