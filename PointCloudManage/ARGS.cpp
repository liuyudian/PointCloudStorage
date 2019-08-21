#include "ARGS.h"
#include <cmath>
#include <ctime>

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
float PI = 3.141592654;
int GetLineLocation(CEdge currentEdge, pcl::PointXYZ point);
int flag = 0;
vector<Surface>surfacelist1;
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
	CEdge cedge;
	Vector3 vector3,neighborpoint;
	Vector3 m, n, q,normal1,normal2;

	pcl::PointXYZ orginpoint;//����Դ��
	vector<Vector3> orginsurface1;
	vector<pcl::PointXYZ>orginsurface;//����������
	//vector<pcl::PointXYZ> neighborpoints;//r�뾶�ڵ��ڵ㼯
	map<float, pcl::PointXYZ>neighborpoints;
	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::io::loadPCDFile<pcl::PointXYZ>("plane.pcd", *cloud);

	if (pcl::io::loadPCDFile<pcl::PointXYZ>("plane.pcd", *cloud) == -1)
	{
		std::cout << "Cloud reading failed��" << std::endl;
		return a;
	}
	pcl::PointXYZ minPt, maxPt;
	// ��ȡ��ֵ
	pcl::getMinMax3D(*cloud, minPt, maxPt);
	float r = 0;
	 r = sqrt((abs(maxPt.x - minPt.x)*abs(maxPt.y - minPt.y)*abs(maxPt.z-minPt.z))/ cloud->points.size());
	std::cout << "r is :"<<r << std::endl;
	do {
		int sign = 0;
		//srand(time(NULL));
		int randnum =1128;
		std::cout << randnum << std::endl;
		//�����ȡ�����е�һ����
		orginpoint = cloud->points[randnum];
		//�鵽�������ڵ��ڵ�
		CCloudOctree CCO;
		neighborpoints = CCO.GetField1(cloud,3, orginpoint);
		std::cout << "�����Ĵ�С is :" << neighborpoints.size() << std::endl;

		//��������������˳�μ���������
		orginsurface.push_back(orginpoint);
		if (neighborpoints.size() > 3) 
		{
			auto it = neighborpoints.begin();
			it++;
			orginsurface.push_back(it->second);
			it++;
			orginsurface.push_back(it->second);
		}
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
		if (distance1 >= (distance2 + distance3) || distance2 >= (distance1 + distance3) || distance3 >= (distance2 + distance1))
		{

			orginsurface.clear();
			orginsurface1.clear();
			neighborpoints.clear();
			continue;//������ߣ��ݹ�������ѡԭ�㣻
		}
		//�жϾ������������������Բ�����Ƿ�����ڵ㼯�е�������
		Vector3 cirfc= solveCenterPointOfCircle(orginsurface1, vector3);
		std::cout << cirfc.x << " " << cirfc.y << " " << cirfc.z << " " << cirfc .circler<< std::endl;
		float r = distance(orginsurface1[0], cirfc);
		map<float, pcl::PointXYZ>::iterator itr = neighborpoints.begin();
		itr++;
		for (itr; itr != neighborpoints.end(); itr++)
		{
			neighborpoint.x = itr->second.x;
			neighborpoint.y = itr->second.y;
			neighborpoint.z = itr->second.z;
			sign = 3;
			if (r >= distance(neighborpoint, cirfc))
			{
				orginsurface.clear();
				orginsurface1.clear();
				neighborpoints.clear();
				sign = 1;
				break;//����������������Բ�ڡ�
			}
			
		}
		if (sign == 1)continue;
		std::cout << "hello" << std::endl;
		if (sign == 3)break;
		/*//�ж��ڵ㼯�ĵ��Ƿ������������ͬһ��
		m = orginsurface1[1] - orginsurface1[0];
		n = orginsurface1[2] - orginsurface1[1];
		normal1 = crossProduct(m, n);//�����εķ�����
		neighborpoint.x = neighborpoints[2].x;
		neighborpoint.y = neighborpoints[2].y;
		neighborpoint.z = neighborpoints[2].z;
		normal2 = vector3 - neighborpoint;//����������Բ�ĵ�����
		//������������֮��ļн�
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
			normal2 = vector3 - neighborpoint;//����������Բ�ĵ�����
			//������������֮��ļн�
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
		
		break;*/
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
// ��ȡ��ѡ�㼯�Լ�������ѵ�
pcl::PointXYZ ARGS::GetCandidate(CEdge currentEdge,Surface surface)
{
	// step1 ���һ�ߵ�����ڵ㼯
	// ��ȡ�����
	vector<pcl::PointXYZ>RPoint;
	// ��R
	float L = 0.0f;
	// ����1��ʾ�۽�
	surface.GetAngle();
	if (surface.angle == 1)
	{
		// ��ȡ��߳�
		L = surface.GetMaxLen();
	}
	else
	{
		L = (surface.edge1.GetLen() + surface.edge2.GetLen() + surface.edge3.GetLen())/3.0;
	}
	// ��ߵ����ĵ�
	L = 1.0f;
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
	// ɾ���������Ӧ�ĵ�
	pcl::PointXYZ bestPoint;
	auto it = ConstMap.begin();
	if (ConstMap.size() == 0)
	{
		flag = 1;
		//std::cout << "bestX: " << bestPoint.x << "bestY: " << bestPoint.y << "bestZ: " << bestPoint.z << std::endl;
		return bestPoint;
	}
	else
	{
		bestPoint = it->second;
		//std::cout << "bestX: " << bestPoint.x << "bestY: " << bestPoint.y << "bestZ: " << bestPoint.z << std::endl;

	}
	

	//std::cout <<"bestX: " <<bestPoint.x <<"bestY: " <<bestPoint.y <<"bestZ: " <<bestPoint.z << std::endl;
	return bestPoint;
}
// �����ѡ�㼯�����ɵ�����Ƭ�ĽǶ��Լ��ܳ�����
void GetAngleMaxAndMin(vector<pcl::PointXYZ>RPoint,CEdge currentEdge)
{
	listLen.clear();
	listAngle.clear();
	ConstMap.clear();
	// �� MAX and Min �Լ�ÿ����ѡ�������ڽǽǶȺ��ܳ�
	for (auto it = RPoint.begin();it != RPoint.end();it++)
	{
		GetAngleAndLen(*it, currentEdge);
	}
	//std::cout << "RPoint: " << RPoint.size() << "listLen: " << listLen.size() << "listAngle: " << listAngle.size() << "ConstAngle1: " << ConstAngle1.size() << std::endl;
	if (RPoint.size() == 0)
	{
		flag = 1;
		return;
	}
	for (int i=0;i<listLen.size()&&i<listAngle.size();)
	{
		// ��ʸ����
		//double angle1 = sin(ConstAngle1[i]);

		// ����ڽǴ���
		double angle2 = abs((listAngle[i] - Anglemin) / (Anglemax - Anglemin));
		// �߳�����
		double ConstEdge = abs((listLen[i]-Lenmin)/(Lenmax-Lenmin));

		double JoinCost =  angle2 + ConstEdge;

		Const.push_back(JoinCost);
		ConstMap.insert(pair<double, pcl::PointXYZ>(JoinCost, RPoint[i]));
		i++;

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
	// ���ݷ������жϷ���
	// �ų����ĵ�
	vector<pcl::PointXYZ>NewRPoint;
	//std::cout << "��ʼRPOINT : " << RPoint.size() << std::endl;
	for (auto it = RPoint.begin();it != RPoint.end();it++)
	{
		pcl::PointXYZ point = *it;
		// �¹����������η�ʸҪ���߹��ɵ������η�ʸ���ڲ�ͬ��
		vector<double>list=getNormal(currentEdge.startNode, currentEdge.endNode, point);
		vector<double>listSurface = getNormal(surface.p0, surface.p1, surface.p2);
		if ((list.size() != 0) && (listSurface.size() != 0))
		{
			if (list[1] * listSurface[1] <0)
			{
				NewRPoint.push_back(point);
				//std::cout << "��:  " << point.x << " " << point.y << " " << point.z << endl;
				//std::cout << "�¹����ķ�����:  " << list[0]<<" "<< list[1]<<" "<<list[2]<<endl;
				//std::cout << "��ǰ������Ƭ��  " << endl;  
				//surface.ToString() ;
				//std::cout << "��ǰ�ߵķ�����:  " << listSurface[0] << " " << listSurface[1] << " " << listSurface[2] << endl;

			}
		}
	
	}
	//std::cout << "����RPOINT : " << NewRPoint.size() << std::endl;

	// ��ʸ֮��ļн��޳�����120�ȵĵ�
	/*ConstAngle1.clear();
	vector<pcl::PointXYZ>NewRPoint1;
	for (auto it = NewRPoint.begin();it != NewRPoint.end();it++)
	{
		// ���������֮��ķ�ʸ
		vector<double>list = getNormal(*it, currentEdge.startNode, currentEdge.endNode);
		// ����surface �ķ�ʸ
		vector<double>listSurface = getNormal(surface.p0, surface.p1, surface.p2);
	    // ���㷨ʸ֮��ļн�
		if (list.size() != 0 && listSurface.size() != 0)
		{
			double angle = acos((list[0] * listSurface[0] + list[1] * listSurface[1] + list[2] * listSurface[2]) /
				sqrt(list[0] * list[0] + list[1] * list[1] + list[2] * list[2])*sqrt(listSurface[0] * listSurface[0] + listSurface[1] * listSurface[1] + listSurface[2] * listSurface[2]))*180/PI;
			if (angle < 120)
			{
				NewRPoint1.push_back(*it);
				ConstAngle1.push_back(angle);
			}
		}
		
	}*/
	return NewRPoint;
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
// �жϵ���ֱ�ߵ���һ��
int GetLineLocation(CEdge currentEdge, pcl::PointXYZ point)
{
	double D = 0;
	double A = currentEdge.endNode.y - currentEdge.startNode.y;
	double B= currentEdge.endNode.x - currentEdge.startNode.x;
	double C = currentEdge.endNode.x*currentEdge.startNode.y - currentEdge.startNode.x*currentEdge.endNode.y;
	D = A * point.x + B * point.y + C;
// �Ҳ�
	if (D > 0)
	{
		flag = 1;
	}
	// ���
	else if (D < 0)
	{
		flag = 0;
	}
	else
	{
		flag = -1;
	}

	return flag;
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
// ��ȡ��ѵ�
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
void ARGS::GetARGS()
{
	// step ������������Ƭ ,��SeedT�������߼��뵽���Ա���
	vector<Surface>list;
	vector<CEdge>Edgelist;
	pcl::PolygonMesh triangles;
	int i = 0;
	int count = 0;
	Surface seedT = SelectSurface();
	
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->addLine(seedT.edge1.startNode, seedT.edge1.endNode, std::to_string(count));
	count++;
	viewer->addLine(seedT.edge2.startNode, seedT.edge2.endNode, std::to_string(count));
	count++;
	viewer->addLine(seedT.edge3.startNode, seedT.edge3.endNode, std::to_string(count));
	count++;
	i = 0;
	list.push_back(seedT);
	// seedT.ToString();
	listSurfce.push_back(seedT);
	activeList.push_back(seedT.edge1);
	listSurfce.push_back(seedT);
	activeList.push_back(seedT.edge2);
	listSurfce.push_back(seedT);
	activeList.push_back(seedT.edge3);
	int countActiveList = 0;
	while (!activeList.empty())
	{
		// ɸѡ��ѵ�
		CEdge currentEdge = activeList.front();
		Edgelist.push_back(currentEdge);
		currentEdge.ToString();
		activeList.pop_front();
		listSurfce[i].ToString();
		// ��ȡ��ѵ�
		pcl::PointXYZ point= GetCandidate(currentEdge, listSurfce[i]);
		if (flag == 1)
		{
			break;
		}
		i++;
		CEdge e1(currentEdge.startNode, point);
		CEdge e2(point, currentEdge.endNode);
		// �½�������Ƭ
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
	std::cout << "������Ƭ ��" << list.size() << endl;
	 i = 0;
	for (auto it = list.begin();it != list.end();it++)
	{
		Surface a = *it;
		viewer->addLine(a.edge1.startNode, a.edge1.endNode, std::to_string(count));
		count++;
		viewer->addLine(a.edge2.startNode, a.edge2.endNode, std::to_string(count));
		count++;
		viewer->addLine(a.edge3.startNode, a.edge3.endNode, std::to_string(count));
		count++;
	}
	/*for (auto it = Edgelist.begin();it != Edgelist.end();it++)
   {
	   viewer->addLine(it->startNode, it->endNode, std::to_string(count));
	   count++;
   }*/

}
vector<Surface> ARGS::Wanggehua()
{
	Surface Orgin, a,c1;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer("test"));
	pcl::PointXYZ bestpoint;
	
	vector<CEdge> activelist;

	list<CEdge>active;
	vector<CEdge> currentlist;
	CEdge fixededge, currentedge;
	Orgin = SelectSurface();
	//Orgin.ToString();
	int i1 = 0;
	int i = 0;


	view->addLine(Orgin.edge1.startNode, Orgin.edge1.endNode, std::to_string(i1));
	i1++;
	view->addLine(Orgin.edge2.startNode, Orgin.edge2.endNode, std::to_string(i1));
	i1++;
	view->addLine(Orgin.edge3.startNode, Orgin.edge3.endNode, std::to_string(i1));
	i1++;
	active.push_front(Orgin.edge3);
	active.push_front(Orgin.edge2);
	surfacelist1.push_back(Orgin);
	active.push_front(Orgin.edge1);

	activelist.push_back(Orgin.edge1);
	surfacelist.push_back(Orgin);
	activelist.push_back(Orgin.edge2);
	surfacelist.push_back(Orgin);

	activelist.push_back(Orgin.edge3);
	surfacelist.push_back(Orgin);
	currentedge = active.front();
	currentlist.push_back(currentedge);
	a = surfacelist.front();
	fixededge = Orgin.edge3;
	c1 = Orgin;
	do
	{
		surfacelist1.push_back(a);
		currentedge.ToString();
		//std::cout << "hello : " <<  std::endl;
		//a.ToString();
 		bestpoint = GetCandidate(currentedge, a);
		if (flag == 1)
		{
			flag = 0;
			if (active.size() != 0||surfacelist.size()!=0)
			{
				active.pop_front();
				surfacelist.pop_front();
				currentedge.startNode = active.front().endNode;
				currentedge.endNode = active.front().startNode;
				continue;
			}
			else
				break;
		}
		CEdge edge2(bestpoint, currentedge.endNode), edge1(currentedge.startNode,bestpoint);
		/*if (flag == 1)
		{
			flag = 0;
			if (activelist.size() != 0)
			{
				activelist.erase(activelist.begin());
				currentedge = activelist[0];
				i--;
				continue;
			}

		}*/
		/*edge1.startNode = currentedge.startNode;
		edge1.endNode = bestpoint;
		edge2.startNode = currentedge.endNode;
		edge2.endNode = bestpoint;*/
		a.edge1 = edge1;
		a.edge2 = edge2;
		a.edge3 = currentedge;
		a.p0 = currentedge.startNode;
		a.p1 = bestpoint;
		a.p2 = currentedge.endNode;
		//std::cout << "�¼����������Ƭ " << std::endl;
		a.ToString();

		//surfacelist.push_back(a);
		//activelist.erase(activelist.begin());
		/*if (active.size() <=0|| surfacelist.size()<=0)
		{
			break;
		}*/
		if(active.size()>0)
		{
		active.pop_front();
		surfacelist.pop_front();
		active.push_front(edge2);
		surfacelist.push_front(a);
		active.push_front(edge1);
		surfacelist.push_front(a);
		}
		else break;

		//activelist.insert(activelist.begin(), edge1);
		//activelist.insert(activelist.begin(), edge2);
		/*if(c1.isWithin(currentedge))
		{
		    �ﵽ�ձ�
			active.pop_front();
			surfacelist.pop_front();
			currentedge = active.front();
        }*/
		currentedge = active.front();
		while (count(currentlist.begin(),currentlist.end(), currentedge)>0)
		{
			active.pop_front();
			surfacelist.pop_front();
			currentedge = active.front();
		}
		a = surfacelist.front();
		currentlist.push_back(currentedge);
		//currentedge = activelist[0];
		// view->addLine(surfacelist[0].edge1.startNode, surfacelist[0].edge1.endNode, std::to_string(i));
		i++;
		std::cout <<"���:"<< i << std::endl;
		//7197
		if (i >20)
		{
			break;
		}
	
	
	} while (!active.empty());
	i = 0;

	for (auto it = surfacelist1.begin();it != surfacelist1.end();it++)
	{
		Surface a = *it;
		view->addLine(a.edge1.startNode, a.edge1.endNode,1,0,0, std::to_string(i1));
		
		i1++;
		view->addLine(a.edge2.startNode, a.edge2.endNode, 0, 1, 0, std::to_string(i1));
		i1++;
		view->addLine(a.edge3.startNode, a.edge3.endNode, 0, 0, 1, std::to_string(i1));
		i1++;
	}
	return surfacelist1;
}

//���Ϊply�ļ�
void ARGS::Saveasply()
{
	map<MyPoint, int>pointlist;
	map<MyPoint, int>pointlist1;
	int i = 0;
	for (auto itr = surfacelist1.begin(); itr != surfacelist1.end(); itr++)
	{
		MyPoint mypoint;
		mypoint.x = itr->p0.x;
		mypoint.y = itr->p0.y;
		mypoint.z = itr->p0.z;
		pointlist.insert(std::pair<MyPoint, int>(mypoint, i));
		mypoint.x = itr->p1.x;
		mypoint.y = itr->p1.y;
		mypoint.z = itr->p1.z;
		pointlist.insert(std::pair<MyPoint, int>(mypoint, i));
		mypoint.x = itr->p2.x;
		mypoint.y = itr->p2.y;
		mypoint.z = itr->p2.z;
		pointlist.insert(std::pair<MyPoint, int>(mypoint, i));
		
	}
	for (map<MyPoint, int>::iterator itr = pointlist.begin(); itr != pointlist.end(); itr++)
	{
		pointlist1.insert(std::pair<MyPoint, int>(itr->first,i));
		i++;
	}
	map<MyPoint, int>::iterator v;
	for (auto itr = surfacelist1.begin(); itr != surfacelist1.end(); itr++)
	{
		MyPoint findpoint;
		findpoint.x = itr->p0.x;
		findpoint.y = itr->p0.y;
		findpoint.z = itr->p0.z;
		v = pointlist1.find(findpoint);
		itr->mp0 = v->second;
		//
		findpoint.x = itr->p1.x;
		findpoint.y = itr->p1.y;
		findpoint.z = itr->p1.z;
		v = pointlist1.find(findpoint);
		itr->mp1 = v->second;
		//
		findpoint.x = itr->p2.x;
		findpoint.y = itr->p2.y;
		findpoint.z = itr->p2.z;
		v = pointlist1.find(findpoint);
		itr->mp2 = v->second;
	}
	//
	string fileName = "bunny.ply";
	ofstream of(fileName.c_str());
	of.precision(std::numeric_limits<double>::digits10);
	//ofstream OpenFile("D:\\vs2017projects\\Task2\\testmap.ply");
	if (of.fail())
	{
		std::cout << "���ļ�����!" << std::endl;
	}
	of << "ply" << endl;
	of << "format ascii 1.0" << endl;
	of << "comment generated by platoply" << endl;
	of << "element vertex " << pointlist1.size()<< endl;
	of << "property float32 x" << endl;
	of << "property float32 y" << endl;
	of << "property float32 z" << endl;
	of << "element face " << surfacelist1.size() << endl;
	of << "property list uint8 int32 vertex_indices" << endl;
	of << "end_header" << endl;
	for (map<MyPoint, int>::iterator itr = pointlist1.begin(); itr != pointlist1.end(); itr++)
	{
		of << " " << itr->first.x << " " << itr->first.y << " " << itr->first.z << endl;
	}
	for (auto itr = surfacelist1.begin(); itr != surfacelist1.end(); itr++)
	{
		of << 3 << " " << itr->mp0 << " " << itr->mp1 << " " << itr->mp2 << endl;
	}

}
