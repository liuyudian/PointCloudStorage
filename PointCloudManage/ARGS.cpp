#include "ARGS.h"

vector<pcl::PointXYZ> DeletFixedPoint(Surface surface, vector<pcl::PointXYZ> RPoint);
double GetAngle(CEdge currentEdge, CEdge otherEdge);

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
	float PI = 3.141592654;
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
	do {
		int sign = 0;
		int randnum = rand() % cloud->points.size();
		std::cout << randnum << std::endl;
		//�����ȡ�����е�һ����
		orginpoint = cloud->points[randnum];
		//�鵽�������ڵ��ڵ�
		CCloudOctree CCO;
		neighborpoints = CCO.GetField1(cloud,3, orginpoint);
		//��������������˳�μ���������
		orginsurface.push_back(orginpoint);
		orginsurface.push_back(neighborpoints[1]);
		orginsurface.push_back(neighborpoints[2]);
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
				break;//����������������Բ�ڣ��ݹ顣
			}
		}
		if (sign == 1)continue;
		std::cout << "hello" << std::endl;
		//�ж��ڵ㼯�ĵ��Ƿ������������ͬһ��
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
		
		break;
	} while (1);
	std::cout << vector3.x << " " << vector3.y << " " << vector3.z << std::endl;
	cedge.startNode = orginsurface[0];
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


// ��ȡ��ѡ�㼯
vector<pcl::PointXYZ> ARGS::GetCandidate(CEdge currentEdge,CEdge frontEdge,CEdge rearEdge, Surface surface)
{
	// step1 ���һ�ߵ�����ڵ㼯

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
	if (RPoint.size() == 0)
	{
		// ���ص�ʱ����Ҫ�жϺ�ѡ�㼯�Ƿ�Ϊ��
		return RPoint;
	}
	// �߽Ƕ�Լ���仯
	// �� ���֮ǰ�Լ�֮��ı����ߵļн�
	return RPoint;
}

// ��Ƕ�
double GetAngle(CEdge currentEdge,CEdge otherEdge)
{
	// �������߶μн�
	double  len1 = (otherEdge.startNode.x - currentEdge.startNode.x);
	return len1;
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
// ��Ӵ���
vector<pcl::PointXYZ> ARGS::AddCost(Surface seedSurface)
{
	return vector<pcl::PointXYZ>();
}
// ��ȡ��ѵ�
vector<pcl::PointXYZ> ARGS::GetBestPointNode(vector<pcl::PointXYZ> candidatePointNode)
{
	return vector<pcl::PointXYZ>();
}
