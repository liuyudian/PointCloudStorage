#pragma once
#include"CEdge.h"
class Surface
{
public:
	Surface();
	~Surface();

	// 边的信息
	CEdge edgenhjgj;
	//Edge edge2;
	//Edge edge3;
	// 三角面片的三角形性质 1 表示钝角， 0表示锐角
	int angle;
	float GetMaxLen();
};

