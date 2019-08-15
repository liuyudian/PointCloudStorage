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

double CEdge::GetLen()
{
	pcl::PointXYZ pointa = startNode;
	pcl::PointXYZ pointb = endNode;
	double len_ab = 0;
	 len_ab = sqrt(pow((pointa.x - pointb.x), 2.0) + pow((pointa.y - pointb.y), 2.0) + pow((pointa.z - pointb.z), 2.0));
	return len_ab;
}
