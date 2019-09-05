#pragma once
//����ͷ�ļ�
#include <pcl/octree/octree.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <fstream>
#include <sstream>
#include <iostream>
#include <map>
#include "MyPoint.h"
/*

	���Ƶ��ļ�����

*/
class FileOption
{
public:

	//����
	map<int, MyPoint> _mapPoint;
	FileOption();
	~FileOption();
	//���ܺ���

	void ReadAscFile(const char *cfilename); //��ȡ.asc�ļ�
	string AscToPcd(); //.asc�ļ�תPcd�ļ�
	void SaveAsPLY(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTriangles,pcl::PolygonMesh triangles); //�ѻ��ֺõ�������Ƭ���Ϊ.ply�ļ�
	void ReadPcd(); // ��ȡPCD�ļ�
	
};

