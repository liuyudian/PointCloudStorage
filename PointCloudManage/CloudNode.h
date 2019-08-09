#pragma once
#include "PointCloudManage.h"
#include <pcl/octree/octree.h>
#include "ARGS.h"
#include <vector>
//using namespace std;
class CloudNode
{
public:
	CloudNode();
	vector<pcl::PointXYZ> GetField();
	~CloudNode();

	// 八叉树点云数据
	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud;
	// 八叉树
	//pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree;
    // 获取领域点
	vector<pcl::PointXYZ> CloudNode:: GetField(float L, pcl::PointXYZ pn);

	// 设置点云以及八叉树
	void SetCloudOctree();
};

