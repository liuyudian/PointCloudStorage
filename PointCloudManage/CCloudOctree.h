#pragma once
#include <pcl/octree/octree.h>
#include "PointCloudManage.h"
#include<vector>
using namespace std;
class CCloudOctree
{
public:
	CCloudOctree();
	~CCloudOctree();

	vector<pcl::PointXYZ> GetField();
	// 获取领域点
	vector<pcl::PointXYZ> CCloudOctree::GetField(float L, pcl::PointXYZ pn);

	// 设置点云以及八叉树
	void SetCloudOctree();
};

