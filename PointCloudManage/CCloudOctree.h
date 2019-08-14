#pragma once
#include <pcl/octree/octree.h>
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include<pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include<vector>
using namespace std;
class CCloudOctree
{
public:

	CCloudOctree();
	~CCloudOctree();

	// 获取领域点
	vector<pcl::PointXYZ> CCloudOctree::GetField(float L, pcl::PointXYZ pn);
	vector<pcl::PointXYZ> CCloudOctree::GetField1(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,float L, pcl::PointXYZ pn);
	// 设置点云以及八叉树
	void SetCloudOctree();
};

