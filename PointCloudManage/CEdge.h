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
	// 判断某一个点是否在这条边中
	bool isIn = 0;
	// 边的长度
	double len;
	// 边所在的三角面片的信息
	int surfaceIndex;
	bool Isexist(pcl::PointXYZ p);
	double GetLen();
	void ToString()
	{
		std::cout << "当前活动边： " << std::endl;
		std::cout << " " << startNode.x << " " << startNode.y << " " << startNode.z << std::endl;
		std::cout << " " << endNode.x << " " << endNode.y << " " << endNode.z << std::endl;
	}

	
};

