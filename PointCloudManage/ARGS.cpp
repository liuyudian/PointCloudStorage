#include "ARGS.h"
#include <cmath>

vector<pcl::PointXYZ> DeletFixedPoint(Surface surface, vector<pcl::PointXYZ> RPoint);
double GetAngleFront(CEdge currentEdge, CEdge otherEdge);
double GetAngleRear(CEdge currentEdge, CEdge otherEdge);
vector<double> getNormal(pcl::PointXYZ p0, pcl::PointXYZ p1, pcl::PointXYZ p2);
vector<pcl::PointXYZ> GetNewCandidatePoint(vector<pcl::PointXYZ>  RPoint, CEdge currentEdge, Surface surface);
int GetPointLineRelation(pcl::PointXYZ point, CEdge currentEdge);
void  GetAngleAndLen(pcl::PointXYZ point, CEdge currentEdge);
void GetAngleMaxAndMin(vector<pcl::PointXYZ>RPoint, CEdge currentEdge);
// 与当前活动边构成的备选三角片中的最大内角以及最小内角
double Anglemax = 0;
double Anglemin = 0;
// 表示与当前活动边构建的三角片周长，Lenmax(最大)，Lenmin(最小)
double Lenmax=0;
double Lenmin=0;

int flag = 0;

// 当前三角候选三角面片的周长
vector<double>listLen;
double currentLen = 0;
// 当前候选三角面片的最大角度
vector<double>listAngle;
double currentAngle = 0;
// 法矢代价
vector<double>ConstAngle1;
// 总的代价
vector<double>Const;
map<double, pcl::PointXYZ>ConstMap;
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
	CEdge cedge;
	Vector3 vector3,neighborpoint;
	Vector3 m, n, q,normal1,normal2;
	float PI = 3.141592654;
	pcl::PointXYZ orginpoint;//搜索源点
	vector<Vector3> orginsurface1;
	vector<pcl::PointXYZ>orginsurface;//种子三角形
	vector<pcl::PointXYZ> neighborpoints;//r半径内的邻点集
	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::io::loadPCDFile<pcl::PointXYZ>("plane.pcd", *cloud);

	if (pcl::io::loadPCDFile<pcl::PointXYZ>("plane.pcd", *cloud) == -1)
	{
		std::cout << "Cloud reading failed。" << std::endl;
		return a;
	}
	std::cout << cloud->points.size() << std::endl;
	do {
		int sign = 0;
		int randnum = rand() % cloud->points.size();
		std::cout << randnum << std::endl;
		//随机获取点云中的一个点
		orginpoint = cloud->points[randnum];
		//查到点领域内的邻点
		CCloudOctree CCO;
		neighborpoints = CCO.GetField1(cloud,3, orginpoint);
		//向种子三角形中顺次加入两个点
		orginsurface.push_back(orginpoint);
		orginsurface.push_back(neighborpoints[1]);
		orginsurface.push_back(neighborpoints[2]);
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
		if (distance1 >= (distance2 + distance3) || distance2 >= (distance1 + distance3) || distance3 >= (distance2 + distance1))
		{

			orginsurface.clear();
			orginsurface1.clear();
			neighborpoints.clear();
			continue;//如果共线，递归重新挑选原点；
		}
		//判断经过三角形三个顶点的圆球内是否包含邻点集中的其他点
		float cirfc = solveCenterPointOfCircle(orginsurface1, vector3);
		std::cout << vector3.x << " " << vector3.y << " " << vector3.z << " " << cirfc << std::endl;
		for (int i = 2; i < neighborpoints.size(); i++)
		{
			neighborpoint.x = neighborpoints[i].x;
			neighborpoint.y = neighborpoints[i].y;
			neighborpoint.z = neighborpoints[i].z;
			if (cirfc <= distance(neighborpoint, vector3))
			{
				orginsurface.clear();
				orginsurface1.clear();
				neighborpoints.clear();
				sign = 1;
				break;//如果点在三角形外接圆内，递归。
			}
		}
		if (sign == 1)continue;
		std::cout << "hello" << std::endl;
		//判断邻点集的点是否在三角行面的同一侧
		m = orginsurface1[1] - orginsurface1[0];
		n = orginsurface1[2] - orginsurface1[1];
		normal1 = crossProduct(m, n);//三角形的法向量
		neighborpoint.x = neighborpoints[2].x;
		neighborpoint.y = neighborpoints[2].y;
		neighborpoint.z = neighborpoints[2].z;
		normal2 = vector3 - neighborpoint;//邻域点与外接圆心的向量
		//计算两个向量之间的夹角
		float nn = normal2 * normal1;
		float mm = distance(normal2, normal1);
		float angle = acos(nn / mm)*(180 / PI);
		if (angle<90 && angle>-90) 
		{
			sign = 1;
		}
		else 
		{
			sign = 0;
		}
		for (int i=3;i<neighborpoints.size();i++) 
		{
			neighborpoint.x = neighborpoints[i].x;
			neighborpoint.y = neighborpoints[i].y;
			neighborpoint.z = neighborpoints[i].z;
			normal2 = vector3 - neighborpoint;//邻域点与外接圆心的向量
			//计算两个向量之间的夹角
			float nn = normal2 * normal1;
			float mm = distance(normal2, normal1);
			float angle = acos(nn / mm)*(180 / PI);
			int sign1;
			if (angle<90 && angle>-90)
			{
				sign1 = 1;
			}
			else
			{
				sign1 = 0;
			}
			if (sign != sign1) 
			{
				orginsurface.clear();
				orginsurface1.clear();
				neighborpoints.clear();
				sign = 3;
				break;
			}
		}
		if (sign == 3) 
		{
			continue;
		}
		
		break;
	} while (1);
	std::cout << vector3.x << " " << vector3.y << " " << vector3.z << std::endl;
	a.p0 = orginsurface[0];
	cedge.startNode = orginsurface[0];
	a.p1 = orginsurface[1];
	a.p2 = orginsurface[2];
	cedge.endNode= orginsurface[1];
	a.edge1 = cedge;
	cedge.startNode = orginsurface[1];
	cedge.endNode = orginsurface[2];
	a.edge2 = cedge;
	cedge.startNode = orginsurface[2];
	cedge.endNode = orginsurface[0];
	a.edge3 = cedge;

	return a;
 }
// 获取候选点集以及查找最佳点
pcl::PointXYZ ARGS::GetCandidate(CEdge currentEdge,Surface surface)
{
	// step1 查找活动边的最近邻点集
	// 获取领域点
	vector<pcl::PointXYZ>RPoint;
	// 求R
	float L = 0.0f;
	// 等于1表示钝角
	surface.GetAngle();
	if (surface.angle == 1)
	{
		// 获取最长边长
		L = surface.GetMaxLen();
	}
	else
	{
		L = (surface.edge1.GetLen() + surface.edge2.GetLen() + surface.edge3.GetLen())/3.0;
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
	// 优化候选点集,剔除大于120度的点
	RPoint = GetNewCandidatePoint(RPoint, currentEdge, surface);
	// 计算代价并排序
	GetAngleMaxAndMin(RPoint,currentEdge);

	auto it = ConstMap.begin();
	pcl::PointXYZ bestPoint = it->second;
	std::cout <<"bestX: " <<bestPoint.x <<"bestY: " <<bestPoint.y <<"bestZ: " <<bestPoint.z << std::endl;
	return bestPoint;
}
// 计算候选点集所构成的三角片的角度以及周长代价
void GetAngleMaxAndMin(vector<pcl::PointXYZ>RPoint,CEdge currentEdge)
{
	listLen.clear();
	listAngle.clear();
	// 求 MAX and Min 以及每个候选点的最大内角角度和周长
	for (auto it = RPoint.begin();it != RPoint.end();it++)
	{
		GetAngleAndLen(*it, currentEdge);
	}
	std::cout << "RPoint: " << RPoint.size() << "listLen: " << listLen.size() << "listAngle: " << listAngle.size() << "ConstAngle1: " << ConstAngle1.size() << std::endl;
	if (RPoint.size() == 0)
	{
		flag = 1;
		return;
	}
	for (int i=0;i<listLen.size()&&i<listAngle.size()&&i< ConstAngle1.size();i++)
	{
		// 法矢代价
		double angle1 = sin(ConstAngle1[i]);

		// 最大内角代价
		double angle2 = abs((listAngle[i] - Anglemin) / (Anglemax - Anglemin));
		// 边长代价
		double ConstEdge = abs((listLen[i]-Lenmin)/(Lenmax-Lenmin));

		double JoinCost = angle1 + angle2 + ConstEdge;
		Const.push_back(JoinCost);
		ConstMap.insert(pair<double, pcl::PointXYZ>(JoinCost, RPoint[i]));
	}

}
// 获取角度以及边长的最值
void  GetAngleAndLen(pcl::PointXYZ point, CEdge currentEdge)
{
	pcl::PointXYZ pointa = currentEdge.startNode;
	pcl::PointXYZ pointb = currentEdge.endNode;

	double len_ab = sqrt(pow((pointa.x - pointb.x), 2.0) + pow((pointa.y - pointb.y), 2.0) + pow((pointa.z - pointb.z), 2.0));

	double len_as = sqrt(pow((pointa.x - point.x), 2.0) + pow((pointa.y - point.y), 2.0) + pow((pointa.z - point.z), 2.0));
	double len_bs = sqrt(pow((point.x - pointb.x), 2.0) + pow((point.y - pointb.y), 2.0) + pow((point.z - pointb.z), 2.0));

	// 周长
     currentLen = len_ab + len_as + len_bs;
	 listLen.push_back(currentLen);

	double A = acos((len_ab*len_ab + len_as * len_as - len_bs * len_bs) / (2 * len_ab*len_as));

	double B = acos((len_ab*len_ab + len_bs * len_bs - len_as * len_as) / (2 * len_ab*len_bs));

	double S = acos((len_as*len_as + len_bs * len_bs - len_ab * len_ab) / (2 * len_as*len_bs));

	
	if (A > B)
	{
		if (A > S)
		{
			currentAngle = A;
		}
		else
		{
			currentAngle = S;
		}
	}
	else
	{
		if (B > S)
		{
			currentAngle = B;
		}
		else
		{
			currentAngle = S;
		}
	}
	listAngle.push_back(currentAngle);
	if (currentLen > Lenmax)
	{
		Lenmax = currentLen;
	}
	if (currentLen < Lenmin)
	{
		Lenmin = currentLen;
	}

	if (currentAngle > Anglemax)
	{
		Anglemax = currentAngle;
	}
	if (currentAngle < Anglemin)
	{
		Anglemin = currentAngle;
	}
} 
// 排除当前活动边左侧的点并排除点与活动边构成的三角片法矢与活动边三角片法矢之间的夹角，大于120度排除
vector<pcl::PointXYZ> GetNewCandidatePoint(vector<pcl::PointXYZ>  RPoint, CEdge currentEdge,Surface surface)
{
	// 排除左侧的点
	vector<pcl::PointXYZ>NewRPoint;
	/*for (auto it = RPoint.begin();it != RPoint.end();it++)
	{
		pcl::PointXYZ point = *it;
		int value=GetPointLineRelation(point, currentEdge);
		if (value == 1)
		{
			NewRPoint.push_back(point);
		}
	}*/
	// 法矢之间的夹角剔除大于120度的点
	ConstAngle1.clear();
	vector<pcl::PointXYZ>NewRPoint1;
	for (auto it = RPoint.begin();it != RPoint.end();it++)
	{
		// 计算点与活动边之间的法矢
		vector<double>list = getNormal(*it, currentEdge.startNode, currentEdge.endNode);
		// 计算surface 的法矢
		vector<double>listSurface = getNormal(surface.p0, surface.p1, surface.p2);
	    // 计算法矢之间的夹角
		if (list.size() != 0 && listSurface.size() != 0)
		{
			double angle = acos((list[0] * listSurface[0] + list[1] * listSurface[1] + list[2] * listSurface[2]) /
				sqrt(list[0] * list[0] + list[1] * list[1] + list[2] * list[2])*sqrt(listSurface[0] * listSurface[0] + listSurface[1] * listSurface[1] + listSurface[2] * listSurface[2]));
			if (angle < 120)
			{
				NewRPoint1.push_back(*it);
				ConstAngle1.push_back(angle);
			}
		}
		
	}
	return NewRPoint1;
}
//三维空间中，判断点与直线的位置关系,0表示点在直线上，1
int GetPointLineRelation(pcl::PointXYZ point,CEdge currentEdge)
{
	int value = 0;
	pcl::PointXYZ pointa= currentEdge.startNode;
	pcl::PointXYZ pointb= currentEdge.endNode;

	double len_ab= sqrt(pow((pointa.x - pointb.x), 2.0) + pow((pointa.y - pointb.y), 2.0) + pow((pointa.z - pointb.z), 2.0));
	
	double len_as = sqrt(pow((pointa.x - point.x), 2.0) + pow((pointa.y - point.y), 2.0) + pow((pointa.z - point.z), 2.0));
	double len_bs = sqrt(pow((point.x - pointb.x), 2.0) + pow((point.y - pointb.y), 2.0) + pow((point.z - pointb.z), 2.0));
	if (len_as > len_bs) {//1
		if (len_as > len_ab) {
			value = 1;
		}
		else {
			value = 0;
		}
	}
	else {//-1
		if (len_bs > len_ab) {
			value = -1;
		}
		else {
			value = 0;
		}
	}
	return value;
}
//三维空间中，判断点是否在三角形内
bool isInTrigon(pcl::PointXYZ point, Surface surface)
{
	bool value = false;
	pcl::PointXYZ pointa = surface.edge1.startNode;
	pcl::PointXYZ pointb = surface.edge1.endNode;
	pcl::PointXYZ pointc = surface.edge2.endNode;

	return value;
	// 点到直线的距离
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
// 获取最佳点
vector<double> getNormal(pcl::PointXYZ p0, pcl::PointXYZ p1, pcl::PointXYZ p2)
{
	// 存放法矢
	vector<double>list;
	double v1x = p1.x - p0.x;
	double v1y = p1.y - p0.y;
	double v1z = p1.z - p0.z;
	double v2x = p2.x - p1.x;
	double v2y = p2.y - p1.y;
	double v2z = p2.z - p1.z;
	double x = v1y * v2z - v1z * v2y;
	double y = v1z * v2x - v1x * v2z;
	double z = v1x * v2y - v1y * v2x;
	double len = sqrt(x*x + y * y + z * z);
	if (len == 0)
	{
		return list;
	}
	else {
		list.push_back(x / len);
		list.push_back(y / len);
		list.push_back(z / len);
	}
	return list;
}
void ARGS::GetARGS()
{
	// step 定义种子三角片 ,将SeedT的三条边加入到活性边中
	vector<Surface>list;
	pcl::PolygonMesh triangles;
	int i = 0;
	Surface seedT = SelectSurface();

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	i = 0;
	list.push_back(seedT);
	// seedT.ToString();
	listSurfce.push_back(seedT);
	activeList.push_back(seedT.edge1);
	listSurfce.push_back(seedT);
	activeList.push_back(seedT.edge2);
	listSurfce.push_back(seedT);
	activeList.push_back(seedT.edge3);
	while (!activeList.empty())
	{
		// 筛选最佳点
		CEdge currentEdge = activeList.front();
		currentEdge.ToString();
		activeList.pop_front();
		listSurfce[i].ToString();
		// 获取最佳点
		pcl::PointXYZ point= GetCandidate(currentEdge, listSurfce[i]);
		if (flag == 1)
		{
			break;
		}
		i++;
		CEdge e1(currentEdge.startNode, point);
		CEdge e2(point, currentEdge.endNode);
		// 新建的三角片
		Surface sf(e1, e2, currentEdge);
		sf.p0 = currentEdge.startNode;
		sf.p1 = point;
		sf.p2 = currentEdge.endNode;
		list.push_back(sf);
		activeList.push_back(e1);
		listSurfce.push_back(sf);
		activeList.push_back(e2);
		listSurfce.push_back(sf);
	}

	std::cout << "三角面片 ：" << list.size() << endl;
	 i = 0;
	for (auto it = list.begin();it != list.end();it++)
	{
		Surface a = *it;
		viewer->addLine(a.edge1.startNode, a.edge1.endNode, std::to_string(i));
		i++;
		viewer->addLine(a.edge2.startNode, a.edge2.endNode, std::to_string(i));
		i++;
		viewer->addLine(a.edge3.startNode, a.edge3.endNode, std::to_string(i));
		i++;
	}
}