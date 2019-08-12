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

Surface ARGS::SelectSurface()
{
	return Surface();
 }

  
// ѡ������������Ƭ
// ��ȡ��ѡ�㼯
vector<pcl::PointXYZ> ARGS::GetCandidate(CEdge currentEdge,CEdge frontEdge,CEdge rearEdge, Surface surface)
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
	if (RPoint.size() == 0)
	{
		// ���ص�ʱ����Ҫ�жϺ�ѡ�㼯�Ƿ�Ϊ��
		return RPoint;
	}
	// �߽Ƕ�Լ���仯
	// �� ���֮ǰ�Լ�֮��ı����ߵļн�
	double angleA=GetAngleFront(currentEdge, frontEdge);
	double angleB =GetAngleRear(currentEdge, rearEdge);

	if (angleB < 135)
	{
		// Ӱ��������±߽磬�жϵ��Ƿ��ڱߵ�һ�࣬rearEdgeΪ�±߽�
		// A=y2-y1 B=x1-x2 C=x2*y1-x1*y2;
		// 
		double D = 0;
		double A = rearEdge.endNode.y - rearEdge.startNode.y;
		double B = rearEdge.startNode.x - rearEdge.endNode.x;
		double C = rearEdge.endNode.x*rearEdge.startNode.y - rearEdge.startNode.x*rearEdge.endNode.y;
		// ������
		RPoint = CCO.GetField(L, centerPoint);

		// �����ڱ߽��ڵĵ�
	}
	if (angleA > 135)
	{
		// ������㼯�е�ÿ�����뵱ǰ��������˵���������ʸ���������߷���֮��ļнǣ��Ƕȴ���135�� �Ӻ�ѡ�㼯��ɾ��
	}
	return RPoint;
}
//��ά�ռ��У��жϵ���ֱ�ߵ�λ�ù�ϵ,0��ʾ����ֱ���ϣ�1
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
//��ά�ռ��У��жϵ��Ƿ�����������
bool isInTrigon(pcl::PointXYZ point, Surface surface)
{
	return false;

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
