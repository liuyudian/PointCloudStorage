#pragma once
#include <pcl/octree/octree.h>
class CEdge
{
public:
	CEdge();
	~CEdge();
	pcl::PointXYZ startNode;
	pcl::PointXYZ endNode;
	// 边的长度
	float len;
	// 边所在的三角面片的信息
	int surfaceIndex;
};

