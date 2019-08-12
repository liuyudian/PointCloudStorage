#include "ARGS.h"
#include <cmath>

vector<pcl::PointXYZ> DeletFixedPoint(Surface surface, vector<pcl::PointXYZ> RPoint);
double GetAngleFront(CEdge currentEdge, CEdge otherEdge);
double GetAngleRear(CEdge currentEdge, CEdge otherEdge);
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
	CCloudOctree CCO;
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
	double angleA=GetAngleFront(currentEdge, frontEdge);
	double angleB =GetAngleRear(currentEdge, rearEdge);

	if (angleB < 135)
	{
		// 影响区域的下边界，判断点是否在边的一侧，rearEdge为下边界
		// A=y2-y1 B=x1-x2 C=x2*y1-x1*y2;
		// 
		double D = 0;
		double A = rearEdge.endNode.y - rearEdge.startNode.y;
		double B = rearEdge.startNode.x - rearEdge.endNode.x;
		double C = rearEdge.endNode.x*rearEdge.startNode.y - rearEdge.startNode.x*rearEdge.endNode.y;
		// 求领域
		RPoint = CCO.GetField(L, centerPoint);

		// 保留在边界内的点
	}
	if (angleA > 135)
	{
		// 计算领点集中的每个点与当前活动边两个端点所构建的矢量方向与活动边方向之间的夹角，角度大于135则 从候选点集中删除
	}
	return RPoint;
}
//三维空间中，判断点与直线的位置关系,0表示点在直线上，1
int GetPointLineRelation(pcl::PointXYZ point,CEdge currentEdge)
{
	int value = 0;
	pcl::PointXYZ pointa=currentEdge.startNode;
	pcl::PointXYZ pointb=currentEdge.endNode;

	double len_ab= sqrt(pow((pointa.x - pointb.x), 2.0) + pow((pointa.y - pointb.y), 2.0) + pow((pointa.z - pointb.z), 2.0));
	
	double len_as = sqrt(pow((pointa.x - point.x), 2.0) + pow((pointa.y - point.y), 2.0) + pow((pointa.z - point.z), 2.0));
	double len_bs = sqrt(pow((point.x - pointb.x), 2.0) + pow((point.y - pointb.y), 2.0) + pow((point.z - pointb.z), 2.0));
	return 0;
}
//三维空间中，判断点是否在三角形内
bool isInTrigon(pcl::PointXYZ point, Surface surface)
{
	return false;

}
// 求与活动边之前的边的角度
double GetAngleFront(CEdge currentEdge,CEdge otherEdge)
{
	// 求两条线段夹角
	// pjpi长度
	double l1 =sqrt((currentEdge.startNode.x - currentEdge.endNode.x)*(currentEdge.startNode.x - currentEdge.endNode.x)+
		(currentEdge.startNode.y- currentEdge.endNode.y)*(currentEdge.startNode.y - currentEdge.endNode.y)+
		(currentEdge.startNode.z - currentEdge.endNode.z)*(currentEdge.startNode.z - currentEdge.endNode.z));
	// papi的长度
	double l2= sqrt((otherEdge.startNode.x - otherEdge.endNode.x)*(otherEdge.startNode.x - otherEdge.endNode.x) +
		(otherEdge.startNode.y - otherEdge.endNode.y)*(otherEdge.startNode.y - otherEdge.endNode.y) +
		(otherEdge.startNode.z - otherEdge.endNode.z)*(otherEdge.startNode.z - otherEdge.endNode.z));
	// papj的长度
	double l3= sqrt((otherEdge.endNode.x - currentEdge.endNode.x)*(otherEdge.endNode.x - currentEdge.endNode.x) +
		(otherEdge.endNode.y - currentEdge.endNode.y)*(otherEdge.endNode.y - currentEdge.endNode.y) +
		(otherEdge.endNode.z - currentEdge.endNode.z)*(otherEdge.endNode.z - currentEdge.endNode.z));

	// 求角度
	double A = acos((l1*l1 + l2 * l2 - l3 * l3) / 2.0*l1*l2);
	return A;
}

// 求与活动边之后的边的角度
double GetAngleRear(CEdge currentEdge, CEdge otherEdge)
{
	// 求两条线段夹角
	// pjpi长度
	double l1 = sqrt((currentEdge.startNode.x - currentEdge.endNode.x)*(currentEdge.startNode.x - currentEdge.endNode.x) +
		(currentEdge.startNode.y - currentEdge.endNode.y)*(currentEdge.startNode.y - currentEdge.endNode.y) +
		(currentEdge.startNode.z - currentEdge.endNode.z)*(currentEdge.startNode.z - currentEdge.endNode.z));
	// pbpj的长度
	double l2 = sqrt((otherEdge.startNode.x - otherEdge.endNode.x)*(otherEdge.startNode.x - otherEdge.endNode.x) +
		(otherEdge.startNode.y - otherEdge.endNode.y)*(otherEdge.startNode.y - otherEdge.endNode.y) +
		(otherEdge.startNode.z - otherEdge.endNode.z)*(otherEdge.startNode.z - otherEdge.endNode.z));
	// pbpi的长度
	double l3 = sqrt((otherEdge.endNode.x - currentEdge.startNode.x)*(otherEdge.endNode.x - currentEdge.startNode.x) +
		(otherEdge.endNode.y - currentEdge.startNode.y)*(otherEdge.endNode.y - currentEdge.startNode.y) +
		(otherEdge.endNode.z - currentEdge.startNode.z)*(otherEdge.endNode.z - currentEdge.startNode.z));

	// 求角度
	double B = acos((l1*l1 + l2 * l2 - l3 * l3) / 2.0*l1*l2);
	return B;
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
