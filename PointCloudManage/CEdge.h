#pragma once
#include <pcl/octree/octree.h>
class CEdge
{
public:
	CEdge();
	~CEdge();
	pcl::PointXYZ startNode;
	pcl::PointXYZ endNode;
	// �ߵĳ���
	float len;
	// �����ڵ�������Ƭ����Ϣ
	int surfaceIndex;
};

