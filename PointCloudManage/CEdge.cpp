#include "CEdge.h"



CEdge::CEdge()
{
}

CEdge::CEdge(pcl::PointXYZ a1, pcl::PointXYZ b1)
{
	this->startNode = a1;
	this->endNode = b1;
}


CEdge::~CEdge()
{
}

bool CEdge::Isexist(pcl::PointXYZ p)
{
	if (startNode.x==p.x&&startNode.y==p.y&&startNode.z==p.z)
	{
		return true;
	}
	else if (endNode.x == p.x&&endNode.y == p.y&&endNode.z == p.z)
	{
		return true;
	}
	return false;
}
