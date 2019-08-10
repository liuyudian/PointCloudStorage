#pragma once
#include "PointCloudManage.h"
#include <pcl/octree/octree.h>
#include "ARGS.h"
#include <vector>
//using namespace std;
class CloudNode
{
public:
	CloudNode();
	vector<pcl::PointXYZ> GetField();
	~CloudNode();

	// �˲�����������
	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud;
	// �˲���
	//pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree;
    // ��ȡ�����
	vector<pcl::PointXYZ> CloudNode:: GetField(float L, pcl::PointXYZ pn);

	// ���õ����Լ��˲���
	void SetCloudOctree();
};

