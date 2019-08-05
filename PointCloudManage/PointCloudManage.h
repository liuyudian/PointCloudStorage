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


//pcl��vtk��ͷ�ļ�
#include <vtkRenderWindow.h>
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include<pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>

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
};
