#pragma once
#include <pcl/octree/octree.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include "CEdge.h"
#include "Surface.h"
#include<vector>
using namespace std;

class ARGS
{
public:

	// ����
	vector<pcl::PointXYZ>candidatePointNode;
	ARGS();
	~ARGS();

	// ѡ����������Ƭ
	//Surface SelectSurface();


	// ��ȡ��ѡ�㼯,��������ߣ�������ڵ�������Ƭ,��������
	//vector<PointNode> GetCandidate(Edge currentEdge, Surface surface, OctreeNode goct);

	// ��ѡ����Ӵ���
	//vector<pcl::PointXYZ> AddCost(Surface seedSurface);

	// ��ѵ�ɸѡ
	//vector<pcl::PointXYZ> GetBestPointNode(vector<pcl::PointXYZ> candidatePointNode);
};

