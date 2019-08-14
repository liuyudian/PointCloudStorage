#pragma once
#include <pcl/octree/octree.h>
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>
#include<pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include<vector>
using namespace std;
class CCloudOctree
{
public:

	CCloudOctree();
	~CCloudOctree();

	// ��ȡ�����
	vector<pcl::PointXYZ> CCloudOctree::GetField(float L, pcl::PointXYZ pn);

	// ���õ����Լ��˲���
	void SetCloudOctree();
};
