#pragma once
#include <pcl/octree/octree.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include "CEdge.h"
#include "Surface.h"
#include "CCloudOctree.h"
#include<vector>
#include"Vector3.h"
using namespace std;

class ARGS
{
public:

	// 特性
	vector<pcl::PointXYZ>candidatePointNode;
	ARGS();
	~ARGS();
	//获取两点之间的距离

	// 选择种子三角片
	Surface SelectSurface();


	// 获取候选点集,参数：活动边，,活动边之前的边，活动边之后的边，活动边所在的三角面片,点云数据
	vector<pcl::PointXYZ> GetCandidate(CEdge currentEdge,Surface surface);

	// 候选点添加代价
	vector<pcl::PointXYZ> AddCost(Surface seedSurface);

	// 最佳点筛选
	vector<pcl::PointXYZ> GetBestPointNode(vector<pcl::PointXYZ> candidatePointNode);
};

