#pragma once
#include"CEdge.h"
class Surface
{
public:
	Surface();
	~Surface();

	// �ߵ���Ϣ
	CEdge edgenhjgj;
	//Edge edge2;
	//Edge edge3;
	// ������Ƭ������������ 1 ��ʾ�۽ǣ� 0��ʾ���
	int angle;
	float GetMaxLen();
};

