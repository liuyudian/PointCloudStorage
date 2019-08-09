#pragma once
#include <pcl/octree/octree.h>
#include "PointCloudManage.h"
#include<vector>
using namespace std;
class CCloudOctree
{
public:
	CCloudOctree();
	~CCloudOctree();

	vector<pcl::PointXYZ> GetField();
	// ��ȡ�����
	vector<pcl::PointXYZ> CCloudOctree::GetField(float L, pcl::PointXYZ pn);

	// ���õ����Լ��˲���
	void SetCloudOctree();
};

