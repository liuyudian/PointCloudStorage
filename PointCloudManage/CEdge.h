#pragma once
#include <pcl/octree/octree.h>
class CEdge
{
public:
	CEdge();
	~CEdge();
	pcl::PointXYZ startNode;
	pcl::PointXYZ endNode;
	// 判断某一个点是否在这条边中
	bool isIn = 0;
	// 边的长度
	float len;
	// 边所在的三角面片的信息
	int surfaceIndex;
	bool Isexist(pcl::PointXYZ p);
};

