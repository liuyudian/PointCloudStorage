#include "CCloudOctree.h"


CCloudOctree::CCloudOctree()
{

}
CCloudOctree::~CCloudOctree()
{

}

// ��ȡ����
vector<pcl::PointXYZ> CCloudOctree::GetField(float L, pcl::PointXYZ pn)
{
	L = 10;
	pn.x = -28.218;
	pn.y = -7.98492;
	pn.z = 0.161011;
	// ��ȡ����㼯
	vector<pcl::PointXYZ>_vectorPointNode;
	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::io::loadPCDFile<pcl::PointXYZ>("bunny.pcd", *cloud);

	if (pcl::io::loadPCDFile<pcl::PointXYZ>("bunny.pcd", *cloud) == -1)
	{
		std::cout << "Cloud reading failed��" << std::endl;
		return _vectorPointNode;
	}
	// ��С���صı߳�
	float resolution = 128.0f;
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);

	octree.setInputCloud(cloud);
	// ��������ƹ����˲���

	octree.addPointsFromInputCloud();
	// ������

	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;

	// ������ɵ�ĳ�������ڿռ�뾶r��Χ�ڵ������
	if (octree.radiusSearch(pn, L, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
	{
		for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
		{
			// �����������
			std::cout << "    " << cloud->points[pointIdxRadiusSearch[i]].x
			<< " " << cloud->points[pointIdxRadiusSearch[i]].y
			<< " " << cloud->points[pointIdxRadiusSearch[i]].z
			// �±�����������ƽ������
			<< " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;

			pcl::PointXYZ pn1(cloud->points[pointIdxRadiusSearch[i]].x, cloud->points[pointIdxRadiusSearch[i]].y, cloud->points[pointIdxRadiusSearch[i]].z);
			_vectorPointNode.push_back(pn1);
		 }
	}
	return _vectorPointNode;
}

// ���õ��ƺͰ˲���
void CCloudOctree::SetCloudOctree()
{

}