#include "CloudNode.h"

CloudNode::~CloudNode()
{
}
CloudNode::CloudNode()
{
	
}

/*vector<pcl::PointXYZ> CloudNode::GetField(float L, pcl::PointXYZ pn)
{
	// ��ȡ����㼯
	vector<pcl::PointXYZ>_vectorPointNode;
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

         }
	}
	return vector<pcl::PointXYZ>();
}*/

// ���õ��ƺͰ˲���
void CloudNode::SetCloudOctree()
{
	
}
