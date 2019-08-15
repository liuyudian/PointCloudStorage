#pragma once
#include <pcl/octree/octree.h>
class CEdge
{
public:
	CEdge();
	CEdge(pcl::PointXYZ a1, pcl::PointXYZ b1);
	~CEdge();
	pcl::PointXYZ startNode;
	pcl::PointXYZ endNode;
	// �ж�ĳһ�����Ƿ�����������
	bool isIn = 0;
	// �ߵĳ���
	float len;
	// �����ڵ�������Ƭ����Ϣ
	int surfaceIndex;
	bool Isexist(pcl::PointXYZ p);
};

