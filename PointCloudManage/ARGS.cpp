#include "ARGS.h"

vector<pcl::PointXYZ> DeletFixedPoint(Surface surface, vector<pcl::PointXYZ> RPoint);
double GetAngle(CEdge currentEdge, CEdge otherEdge);

ARGS::ARGS()
{
}

ARGS::~ARGS()
{
}

// 选择种子三角面片
Surface ARGS::SelectSurface()
{
	Surface a;
	Vector3 vector3,neighborpoint;
	pcl::PointXYZ orginpoint;//搜索源点
	vector<Vector3> orginsurface1;
	vector<pcl::PointXYZ>orginsurface;//种子三角形
	vector<pcl::PointXYZ> neighborpoints;//r半径内的邻点集
	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::io::loadPCDFile<pcl::PointXYZ>("bunny.pcd", *cloud);

	if (pcl::io::loadPCDFile<pcl::PointXYZ>("bunny.pcd", *cloud) == -1)
	{
		std::cout << "Cloud reading failed。" << std::endl;
		return a;
	}
	std::cout << cloud->points.size() << std::endl;
	int randnum = rand() % cloud->points.size();
	std::cout << randnum << std::endl;
	//随机获取点云中的一个点
	orginpoint = cloud->points[randnum];
	//查到点领域内的邻点
	CCloudOctree CCO;
	neighborpoints = CCO.GetField(8.0, orginpoint);
	//向种子三角形中顺次加入两个点
	orginsurface.push_back(orginpoint);
	orginsurface.push_back(neighborpoints[0]);
	orginsurface.push_back(neighborpoints[1]);
	//判断三个点是否在一条直线上
	for (vector<pcl::PointXYZ>::iterator it = orginsurface.begin(); it != orginsurface.end(); it++) 
	{
		vector3.x = it->x;
		vector3.y = it->y;
		vector3.z = it->z;
		orginsurface1.push_back(vector3);
	}
	float distance1, distance2, distance3;
	distance1 = distance(orginsurface1[0], orginsurface1[1]);
	distance2 = distance(orginsurface1[1], orginsurface1[2]);
	distance3 = distance(orginsurface1[0], orginsurface1[2]);
	if (distance1 >=(distance2 + distance3) && distance2 >= (distance1 + distance3) && distance3 >= (distance2 + distance1))
	{
		SelectSurface();//如果共线，递归重新挑选原点；
	}
	//判断经过三角形三个顶点的圆球内是否包含邻点集中的其他点
	solveCenterPointOfCircle(orginsurface1, vector3);
	std::cout << vector3.x << " " << vector3.y << " " << vector3.z << " " << vector3.r << std::endl;
	for (int i = 2; i < neighborpoints.size(); i++) 
	{
		neighborpoint.x=neighborpoints[i].x;
		neighborpoint.y= neighborpoints[i].y;
		neighborpoint.z = neighborpoints[i].z;
		if (vector3.r <= distance(neighborpoint, vector3)) 
		{
			SelectSurface();//如果点在三角形外接圆内，递归。
		}
	}
	std::cout << "hello" << std::endl;
	//判断邻点集的点是否在三角行面的同一侧



	return Surface();
 }


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

// 求角度
double GetAngle(CEdge currentEdge,CEdge otherEdge)
{
	// 求两条线段夹角
	double  len1 = (otherEdge.startNode.x - currentEdge.startNode.x);
	return len1;
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
