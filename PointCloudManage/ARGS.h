#pragma once
#include <pcl/octree/octree.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include "CEdge.h"
#include "Surface.h"
#include "CCloudOctree.h"
#include<vector>
#include"Vector3.h"
#include<math.h>
#include"MyPoint.h"
using namespace std;

class ARGS
{
public:

	// ����
	vector<pcl::PointXYZ>candidatePointNode;

	list<Surface> surfacelist;//�������񻯵�ÿ��������
	vector<Surface>surfacelist1;
	// ��ߴ洢
	list<CEdge>activeList;
    // �洢������Ƭ
	vector<Surface>listSurfce;
	ARGS();
	~ARGS();
	//��ȡ����֮��ľ���

	// ѡ����������Ƭ
	Surface SelectSurface();


	// ��ȡ��ѡ�㼯,��������ߣ�,���֮ǰ�ıߣ����֮��ıߣ�������ڵ�������Ƭ,��������
	pcl::PointXYZ GetCandidate(CEdge currentEdge,Surface surface);

	// ARGS�㷨
	void GetARGS();

	//��������
	vector<Surface> ArgsAlgorithm();

	//���Ϊply�ļ�
	void Saveasply();

};

