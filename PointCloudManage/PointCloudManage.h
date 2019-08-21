#pragma once
//�Զ������ļ�
#include"FileOption.h"

//QT��ͷ�ļ�
#include <QtWidgets/QMainWindow>
#include "ui_PointCloudManage.h"
#include <QFileInfo>//qt�ļ�����ͷ�ļ�
#include <QMainWindow>
#include <QApplication>
#include <QFileDialog>
#include <QDebug>
#include <QString>
#include <QAction>


//pcl��vtk��ͷ�ļ�
#include <vtkRenderWindow.h>
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include<pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
//�ռ仮��ͷ�ļ�
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
	void ShowModel(); //���ļ�
	void VTK_Show(string s);//��vtk�ؼ�����ʾ����
	void SaveAsPLY();//�����յ�ģ�����Ϊply�ļ�
	void GetLeafShow();
	void Triangulation();//���������ʷ�
	void MeshGeneration();//�Զ��������ʷ�
	void Gridding();//����������Ƭ��ȡ
};
