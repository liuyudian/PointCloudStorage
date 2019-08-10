#include "ARGS.h"
ARGS::ARGS()
{
}

ARGS::~ARGS()
{
}

Surface ARGS::SelectSurface()
{
	return Surface();
 }

  
// 选择种子三角面片
// 获取候选点集
vector<pcl::PointXYZ> ARGS::GetCandidate(CEdge currentEdge,CEdge frontEdge,CEdge rearEdge, Surface surface)
{
	// step1 查找活动边的最近邻点集

	// 获取领域点
	vector<pcl::PointXYZ>RPoint;
	// 求R
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
	// 求边的中心点

	pcl::PointXYZ centerPoint;
	centerPoint.x = (currentEdge.startNode.x + currentEdge.endNode.x) / 2.0;

	centerPoint.y = (currentEdge.startNode.y + currentEdge.endNode.y) / 2.0;

	centerPoint.z = (currentEdge.startNode.z + currentEdge.endNode.z) / 2.0;
	// 获取r=L半径内的领域点
	CCloudOctree cco;
	// 获取领域内的点集
	RPoint = cco.GetField(L, centerPoint);
	if (RPoint.size() == 0)
	{
		// 返回的时候需要判断候选点集是否为空
		return RPoint;
	}
	// 边角度约束间化
	// 求 活动边之前以及之后的边与活动边的夹角


	return RPoint;
}
double GetAngle(CEdge currentEdge,CEdge otherEdge)
{
	// 求两条线段夹角
	double  len1= (otherEdge.startNode.x-currentEdge.startNode.x)*(other)

}
// 固定点删除
vector<pcl::PointXYZ> DeletFixedPoint(Surface surface,vector<pcl::PointXYZ> RPoint)
{
	vector<pcl::PointXYZ> NewRPoint;
	for (auto it = RPoint.begin();it != RPoint.end();)
	{
		// 删除指定的固定点
		if (surface.edge1.Isexist(*it)|| surface.edge2.Isexist(*it)|| surface.edge3.Isexist(*it))
		{
			it++;
		}
		else
		{
			NewRPoint.push_back(*it);
		}
		// 删除排除点
		
	}
	return NewRPoint;
}
// 添加代价
vector<pcl::PointXYZ> ARGS::AddCost(Surface seedSurface)
{
	return vector<pcl::PointXYZ>();
}
// 获取最佳点
vector<pcl::PointXYZ> ARGS::GetBestPointNode(vector<pcl::PointXYZ> candidatePointNode)
{
	return vector<pcl::PointXYZ>();
}
