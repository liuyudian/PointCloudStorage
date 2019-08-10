#include "CloudNode.h"

CloudNode::~CloudNode()
{
}
CloudNode::CloudNode()
{
	
}

/*vector<pcl::PointXYZ> CloudNode::GetField(float L, pcl::PointXYZ pn)
{
	// 获取领域点集
	vector<pcl::PointXYZ>_vectorPointNode;
	// 索引点

	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;

	// 随机生成的某个顶点在空间半径r范围内的领域点
	if (octree.radiusSearch(pn, L, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
	{
		for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
		{ 
			// 搜索点的坐标
			std::cout << "    " << cloud->points[pointIdxRadiusSearch[i]].x
			<< " " << cloud->points[pointIdxRadiusSearch[i]].y
			<< " " << cloud->points[pointIdxRadiusSearch[i]].z
			// 下标点与搜索点的平方距离
			<< " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;

			pcl::PointXYZ pn1(cloud->points[pointIdxRadiusSearch[i]].x, cloud->points[pointIdxRadiusSearch[i]].y, cloud->points[pointIdxRadiusSearch[i]].z);

         }
	}
	return vector<pcl::PointXYZ>();
}*/

// 设置点云和八叉树
void CloudNode::SetCloudOctree()
{
	
}
