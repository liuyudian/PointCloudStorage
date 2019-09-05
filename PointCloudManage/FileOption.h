#pragma once
//导入头文件
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

	点云的文件操作

*/
class FileOption
{
public:

	//特性
	map<int, MyPoint> _mapPoint;
	FileOption();
	~FileOption();
	//功能函数

	void ReadAscFile(const char *cfilename); //读取.asc文件
	string AscToPcd(); //.asc文件转Pcd文件
	void SaveAsPLY(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTriangles,pcl::PolygonMesh triangles); //把划分好的三角面片另存为.ply文件
	void ReadPcd(); // 读取PCD文件
	
};

