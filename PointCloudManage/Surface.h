#pragma once
#include"CEdge.h"
#include <vector>
using namespace std;
class Surface
{
public:
	Surface();
	Surface(CEdge edge1,CEdge edge2,CEdge edge3);
	~Surface();
	// �����Ϣ
	pcl::PointXYZ p0;
	pcl::PointXYZ p1;
	pcl::PointXYZ p2;
	// �ߵ���Ϣ
	CEdge edge1;
	CEdge edge2;
	CEdge edge3;
	// ������Ƭ������������ 1 ��ʾ�۽ǣ� 0��ʾ���
	vector<double>list;

	int angle=0;
	float GetMaxLen();

	// �жϵ��Ƿ���������Ƭ��,???????ƽ�����ά�кβ�ͬ
	bool isWithin(pcl::PointXYZ p);
	void ToString()
	{
		std::cout<<"��ǰ������Ƭ "<<std::endl;
		std::cout <<" " <<p0.x <<" "<<p0.y<<" "<<p0.z<< std::endl;
		std::cout << " " << p1.x << " " << p1.y << " " << p1.z << std::endl;
		std::cout << " " << p2.x << " " << p2.y << " " << p2.z << std::endl;
	}

	// ����������Ƭ�ķ�����
	void GetAngle();
	void Normal();
};

