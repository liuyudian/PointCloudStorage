#pragma once
#include"CEdge.h"
class Surface
{
public:
	Surface();
	~Surface();

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

