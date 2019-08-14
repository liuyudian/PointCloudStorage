#pragma once
#include"CEdge.h"
class Surface
{
public:
	Surface();
	~Surface();
	// 点的信息
	pcl::PointXYZ p0;
	pcl::PointXYZ p1;
	pcl::PointXYZ p2;
	// 边的信息
	CEdge edge1;
	CEdge edge2;
	CEdge edge3;
	// 三角面片的三角形性质 1 表示钝角， 0表示锐角
	int angle;
	float GetMaxLen();

	// 判断点是否在三角面片中,???????平面和三维有何不同
	bool isWithin(pcl::PointXYZ p);
};

