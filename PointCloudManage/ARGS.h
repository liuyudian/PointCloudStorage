#pragma once
#include <pcl/octree/octree.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include "CEdge.h"
#include "Surface.h"
#include "CCloudOctree.h"
#include<vector>
#include"Vector3.h"
#include<math.h>
#include"MyPoint.h"
using namespace std;

class ARGS
{
public:

	// 特性
	vector<pcl::PointXYZ>candidatePointNode;

	list<Surface> surfacelist;//三角网格化的每个三角面
	vector<Surface>surfacelist1;
	// 活动边存储
	list<CEdge>activeList;
    // 存储三角面片
	vector<Surface>listSurfce;
	ARGS();
	~ARGS();
	//获取两点之间的距离

	// 选择种子三角片
	Surface SelectSurface();


	// 获取候选点集,参数：活动边，,活动边之前的边，活动边之后的边，活动边所在的三角面片,点云数据
	pcl::PointXYZ GetCandidate(CEdge currentEdge,Surface surface);

	// ARGS算法
	void GetARGS();

	//点云网格化
	vector<Surface> ArgsAlgorithm();

	//另存为ply文件
	void Saveasply();

};

