#pragma once
#include"CEdge.h"
class Surface
{
public:
	Surface();
	~Surface();
	// �����Ϣ
	pcl::PointXYZ p0;
	pcl::PointXYZ p1;
	pcl::PointXYZ p2;
	// �ߵ���Ϣ
	CEdge edge1;
	CEdge edge2;
	CEdge edge3;
	// ������Ƭ������������ 1 ��ʾ�۽ǣ� 0��ʾ���
	int angle;
	float GetMaxLen();

	// �жϵ��Ƿ���������Ƭ��,???????ƽ�����ά�кβ�ͬ
	bool isWithin(pcl::PointXYZ p);
};

