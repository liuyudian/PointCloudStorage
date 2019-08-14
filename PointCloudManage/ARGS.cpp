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
// �뵱ǰ��߹��ɵı�ѡ����Ƭ�е�����ڽ��Լ���С�ڽ�
double Anglemax = 0;
double Anglemin = 0;
// ��ʾ�뵱ǰ��߹���������Ƭ�ܳ���Lenmax(���)��Lenmin(��С)
double Lenmax=0;
double Lenmin=0;

// ��ǰ���Ǻ�ѡ������Ƭ���ܳ�
vector<double>listLen;
double currentLen = 0;
// ��ǰ��ѡ������Ƭ�����Ƕ�
vector<double>listAngle;
double currentAngle = 0;
// ��ʸ����
vector<double>ConstAngle1;
// �ܵĴ���
vector<double>Const;
map<double, pcl::PointXYZ>ConstMap;
ARGS::ARGS()
{

}

ARGS::~ARGS()
{

}

// ѡ������������Ƭ
Surface ARGS::SelectSurface()
{
	Surface a;
	Vector3 vector3,neighborpoint;
	pcl::PointXYZ orginpoint;//����Դ��
	vector<Vector3> orginsurface1;
	vector<pcl::PointXYZ>orginsurface;//����������
	vector<pcl::PointXYZ> neighborpoints;//r�뾶�ڵ��ڵ㼯
	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::io::loadPCDFile<pcl::PointXYZ>("bunny.pcd", *cloud);

	if (pcl::io::loadPCDFile<pcl::PointXYZ>("bunny.pcd", *cloud) == -1)
	{
		std::cout << "Cloud reading failed��" << std::endl;
		return a;
	}
	std::cout << cloud->points.size() << std::endl;
	int randnum = rand() % cloud->points.size();
	std::cout << randnum << std::endl;
	//�����ȡ�����е�һ����
	orginpoint = cloud->points[randnum];
	//�鵽�������ڵ��ڵ�
	CCloudOctree CCO;
	neighborpoints = CCO.GetField(8.0, orginpoint);
	//��������������˳�μ���������
	orginsurface.push_back(orginpoint);
	orginsurface.push_back(neighborpoints[0]);
	orginsurface.push_back(neighborpoints[1]);
	//�ж��������Ƿ���һ��ֱ����
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
		SelectSurface();//������ߣ��ݹ�������ѡԭ�㣻
	}
	//�жϾ������������������Բ�����Ƿ�����ڵ㼯�е�������
	solveCenterPointOfCircle(orginsurface1, vector3);
	std::cout << vector3.x << " " << vector3.y << " " << vector3.z << " " << vector3.r << std::endl;
	for (int i = 2; i < neighborpoints.size(); i++) 
	{
		neighborpoint.x=neighborpoints[i].x;
		neighborpoint.y= neighborpoints[i].y;
		neighborpoint.z = neighborpoints[i].z;
		if (vector3.r <= distance(neighborpoint, vector3)) 
		{
			SelectSurface();//����������������Բ�ڣ��ݹ顣
		}
	}
	std::cout << "hello" << std::endl;
	//�ж��ڵ㼯�ĵ��Ƿ������������ͬһ��

	return Surface();
 }
// ��ȡ��ѡ�㼯
vector<pcl::PointXYZ> ARGS::GetCandidate(CEdge currentEdge,Surface surface)
{
	// step1 ���һ�ߵ�����ڵ㼯
	CCloudOctree CCO;
	// ��ȡ�����
	vector<pcl::PointXYZ>RPoint;
	// ��R
	float L = 0.0f;
	// ����1��ʾ�۽�
	if (surface.angle == 1)
	{
		// ��ȡ��߳�
		L = surface.GetMaxLen();
	}
	else
	{
		L = (surface.edge1.len + surface.edge2.len + surface.edge3.len)/3.0;
	}
	// ��ߵ����ĵ�

	pcl::PointXYZ centerPoint;
	centerPoint.x = (currentEdge.startNode.x + currentEdge.endNode.x) / 2.0;

	centerPoint.y = (currentEdge.startNode.y + currentEdge.endNode.y) / 2.0;

	centerPoint.z = (currentEdge.startNode.z + currentEdge.endNode.z) / 2.0;
	// ��ȡr=L�뾶�ڵ������
	CCloudOctree cco;
	// ��ȡ�����ڵĵ㼯
	RPoint = cco.GetField(L, centerPoint);
	// �Ż���ѡ�㼯,�޳�����120�ȵĵ�
	RPoint = GetNewCandidatePoint(RPoint, currentEdge, surface);
	// ������۲�����
	GetAngleMaxAndMin(RPoint,currentEdge);

	return RPoint;
}
// �����ѡ�㼯�����ɵ�����Ƭ�ĽǶ��Լ��ܳ�����
void GetAngleMaxAndMin(vector<pcl::PointXYZ>RPoint,CEdge currentEdge)
{

	// �� MAX and Min �Լ�ÿ����ѡ�������ڽǽǶȺ��ܳ�
	for (auto it = RPoint.begin();it != RPoint.end();it++)
	{
		GetAngleAndLen(*it, currentEdge);
	}

	for (int i=0;i<listLen.size();i++)
	{
		// ��ʸ����
		double angle1 = sin(ConstAngle1[i]);

		// ����ڽǴ���
		double angle2 = abs((listAngle[i] - Anglemin) / (Anglemax - Anglemin));
		// �߳�����
		double ConstEdge = abs((listLen[i]-Lenmin)/(Lenmax-Lenmin));

		double JoinCost = angle1 + angle2 + ConstEdge;
		Const.push_back(JoinCost);
		ConstMap.insert(pair<double, pcl::PointXYZ>(JoinCost, RPoint[i]));
	}

}
// ��ȡ�Ƕ��Լ��߳�����ֵ
void  GetAngleAndLen(pcl::PointXYZ point, CEdge currentEdge)
{
	pcl::PointXYZ pointa = currentEdge.startNode;
	pcl::PointXYZ pointb = currentEdge.endNode;

	double len_ab = sqrt(pow((pointa.x - pointb.x), 2.0) + pow((pointa.y - pointb.y), 2.0) + pow((pointa.z - pointb.z), 2.0));

	double len_as = sqrt(pow((pointa.x - point.x), 2.0) + pow((pointa.y - point.y), 2.0) + pow((pointa.z - point.z), 2.0));
	double len_bs = sqrt(pow((point.x - pointb.x), 2.0) + pow((point.y - pointb.y), 2.0) + pow((point.z - pointb.z), 2.0));

	// �ܳ�
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
// �ų���ǰ������ĵ㲢�ų������߹��ɵ�����Ƭ��ʸ��������Ƭ��ʸ֮��ļнǣ�����120���ų�
vector<pcl::PointXYZ> GetNewCandidatePoint(vector<pcl::PointXYZ>  RPoint, CEdge currentEdge,Surface surface)
{
	// �ų����ĵ�
	vector<pcl::PointXYZ>NewRPoint;
	for (auto it = RPoint.begin();it != RPoint.end();it++)
	{
		pcl::PointXYZ point = *it;
		int value=GetPointLineRelation(point, currentEdge);
		if (value == 1)
		{
			NewRPoint.push_back(point);
		}
	}
	// ��ʸ֮��ļн��޳�����120�ȵĵ�
	vector<pcl::PointXYZ>NewRPoint1;
	for (auto it = NewRPoint.begin();it != NewRPoint.end();it++)
	{
		// ���������֮��ķ�ʸ
		vector<double>list = getNormal(*it, currentEdge.startNode, currentEdge.endNode);
		// ����surface �ķ�ʸ
		vector<double>listSurface = getNormal(surface.p0, surface.p1, surface.p2);
	    // ���㷨ʸ֮��ļн�
		double angle = acos((list[0] * listSurface[0] + list[1] * listSurface[1] + list[2] * listSurface[2]) / 
			sqrt(list[0] * list[0] + list[1] * list[1] + list[2] * list[2])*sqrt(listSurface[0] * listSurface[0] + listSurface[1] * listSurface[1] + listSurface[2] * listSurface[2]));
		if (angle < 120)
		{
			NewRPoint1.push_back(*it);
			ConstAngle1.push_back(angle);
		}
	}
	return NewRPoint1;
}
//��ά�ռ��У��жϵ���ֱ�ߵ�λ�ù�ϵ,0��ʾ����ֱ���ϣ�1
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
//��ά�ռ��У��жϵ��Ƿ�����������
bool isInTrigon(pcl::PointXYZ point, Surface surface)
{
	bool value = false;
	pcl::PointXYZ pointa = surface.edge1.startNode;
	pcl::PointXYZ pointb = surface.edge1.endNode;
	pcl::PointXYZ pointc = surface.edge2.endNode;

	return value;
	// �㵽ֱ�ߵľ���
}

// ������֮ǰ�ıߵĽǶ�
double GetAngleFront(CEdge currentEdge,CEdge otherEdge)
{
	// �������߶μн�
	// pjpi����
	double l1 =sqrt((currentEdge.startNode.x - currentEdge.endNode.x)*(currentEdge.startNode.x - currentEdge.endNode.x)+
		(currentEdge.startNode.y- currentEdge.endNode.y)*(currentEdge.startNode.y - currentEdge.endNode.y)+
		(currentEdge.startNode.z - currentEdge.endNode.z)*(currentEdge.startNode.z - currentEdge.endNode.z));
	// papi�ĳ���
	double l2= sqrt((otherEdge.startNode.x - otherEdge.endNode.x)*(otherEdge.startNode.x - otherEdge.endNode.x) +
		(otherEdge.startNode.y - otherEdge.endNode.y)*(otherEdge.startNode.y - otherEdge.endNode.y) +
		(otherEdge.startNode.z - otherEdge.endNode.z)*(otherEdge.startNode.z - otherEdge.endNode.z));
	// papj�ĳ���
	double l3= sqrt((otherEdge.endNode.x - currentEdge.endNode.x)*(otherEdge.endNode.x - currentEdge.endNode.x) +
		(otherEdge.endNode.y - currentEdge.endNode.y)*(otherEdge.endNode.y - currentEdge.endNode.y) +
		(otherEdge.endNode.z - currentEdge.endNode.z)*(otherEdge.endNode.z - currentEdge.endNode.z));

	// ��Ƕ�
	double A = acos((l1*l1 + l2 * l2 - l3 * l3) / 2.0*l1*l2);
	return A;
}

// ������֮��ıߵĽǶ�
double GetAngleRear(CEdge currentEdge, CEdge otherEdge)
{
	// �������߶μн�
	// pjpi����
	double l1 = sqrt((currentEdge.startNode.x - currentEdge.endNode.x)*(currentEdge.startNode.x - currentEdge.endNode.x) +
		(currentEdge.startNode.y - currentEdge.endNode.y)*(currentEdge.startNode.y - currentEdge.endNode.y) +
		(currentEdge.startNode.z - currentEdge.endNode.z)*(currentEdge.startNode.z - currentEdge.endNode.z));
	// pbpj�ĳ���
	double l2 = sqrt((otherEdge.startNode.x - otherEdge.endNode.x)*(otherEdge.startNode.x - otherEdge.endNode.x) +
		(otherEdge.startNode.y - otherEdge.endNode.y)*(otherEdge.startNode.y - otherEdge.endNode.y) +
		(otherEdge.startNode.z - otherEdge.endNode.z)*(otherEdge.startNode.z - otherEdge.endNode.z));
	// pbpi�ĳ���
	double l3 = sqrt((otherEdge.endNode.x - currentEdge.startNode.x)*(otherEdge.endNode.x - currentEdge.startNode.x) +
		(otherEdge.endNode.y - currentEdge.startNode.y)*(otherEdge.endNode.y - currentEdge.startNode.y) +
		(otherEdge.endNode.z - currentEdge.startNode.z)*(otherEdge.endNode.z - currentEdge.startNode.z));

	// ��Ƕ�
	double B = acos((l1*l1 + l2 * l2 - l3 * l3) / 2.0*l1*l2);
	return B;
}
// �̶���ɾ��
vector<pcl::PointXYZ> DeletFixedPoint(Surface surface,vector<pcl::PointXYZ> RPoint)
{
	vector<pcl::PointXYZ> NewRPoint;
	for (auto it = RPoint.begin();it != RPoint.end();)
	{
		// ɾ��ָ���Ĺ̶���
		if (surface.edge1.Isexist(*it)|| surface.edge2.Isexist(*it)|| surface.edge3.Isexist(*it))
		{
			it++;
		}
		else
		{
			NewRPoint.push_back(*it);
		}
		// ɾ���ų���
	}
	return NewRPoint;
}
// ���Ӵ���
vector<pcl::PointXYZ> ARGS::AddCost(Surface seedSurface)
{
	return vector<pcl::PointXYZ>();
}
// ��ȡ��ѵ�
vector<pcl::PointXYZ> ARGS::GetBestPointNode(vector<pcl::PointXYZ> candidatePointNode)
{
	// ���Ӵ��ۣ�������

	return vector<pcl::PointXYZ>();
}
// ��ʸ�Ƕȴ���CostAngle1
double GetConstAngle1()
{
	return 0;
}
// ���㷨ʸ
vector<double> getNormal(pcl::PointXYZ p0, pcl::PointXYZ p1, pcl::PointXYZ p2)
{
	// ��ŷ�ʸ
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