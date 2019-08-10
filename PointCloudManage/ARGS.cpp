#include "ARGS.h"
ARGS::ARGS()
{
}

ARGS::~ARGS()
{
}

//Surface ARGS::SelectSurface()
//{
//	return Surface();
//}

  
// 选择种子三角面片


// 获取候选点集
/*vector<PointNode> ARGS::GetCandidate(Edge currentEdge, Surface surface, OctreeNode goct)
{
	// step1 查找活动边的最近邻点集

	float L = 0.0f;
	// 等于1表示钝角
	if (surface.angle == 1)
	{
		// 获取最长边长
		L = surface.GetMaxLen();
	}
	else
	{
		L = (surface.edge1.len + surface.edge2.len + surface.edge3.len)/3.0;
	}
	// 获取r=L半径内的领域点
	pcl::PointXYZ searchPoint;
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;

	return vector<PointNode>();
}*/
// 添加代价
/*vector<pcl::PointXYZ> ARGS::AddCost(Surface seedSurface)
{
	return vector<pcl::PointXYZ>();
}
// 获取最佳点
vector<pcl::PointXYZ> ARGS::GetBestPointNode(vector<pcl::PointXYZ> candidatePointNode)
{
	return vector<pcl::PointXYZ>();
}*/
