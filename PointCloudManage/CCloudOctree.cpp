#include "CCloudOctree.h"


CCloudOctree::CCloudOctree()
{

}
CCloudOctree::~CCloudOctree()
{

}

// 获取领域
vector<pcl::PointXYZ> CCloudOctree::GetField(float L, pcl::PointXYZ pn)
{
	
	// 获取领域点集
	vector<pcl::PointXYZ>_vectorPointNode;
	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::io::loadPCDFile<pcl::PointXYZ>("plane.pcd", *cloud);

	if (pcl::io::loadPCDFile<pcl::PointXYZ>("plane.pcd", *cloud) == -1)
	{
		std::cout << "Cloud reading failed。" << std::endl;
		return _vectorPointNode;
	}
	// 最小体素的边长
	float resolution = 128.0f;
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);

	octree.setInputCloud(cloud);
	// 从输入点云构建八叉树

	octree.addPointsFromInputCloud();
	// 索引点

	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;

	// 随机生成的某个顶点在空间半径r范围内的领域点
	if (octree.radiusSearch(pn, L, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
	{
		for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
		{
			// 搜索点的坐标
			//std::cout << "    " << cloud->points[pointIdxRadiusSearch[i]].x
			//<< " " << cloud->points[pointIdxRadiusSearch[i]].y
			//<< " " << cloud->points[pointIdxRadiusSearch[i]].z
			// 下标点与搜索点的平方距离
			//<< " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;

			pcl::PointXYZ pn1(cloud->points[pointIdxRadiusSearch[i]].x, cloud->points[pointIdxRadiusSearch[i]].y, cloud->points[pointIdxRadiusSearch[i]].z);
			_vectorPointNode.push_back(pn1);
		 }
	}
	return _vectorPointNode;
}

// 获取领域
map<float, pcl::PointXYZ> CCloudOctree::GetField1(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float L, pcl::PointXYZ pn)
{
	// 获取领域点集
	//vector<pcl::PointXYZ>_vectorPointNode;
	map<float, pcl::PointXYZ>_vectorPointNode;
	// 最小体素的边长
	float resolution = 128.0f;
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);

	octree.setInputCloud(cloud);
	// 从输入点云构建八叉树

	octree.addPointsFromInputCloud();
	// 索引点

	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;

	// 随机生成的某个顶点在空间半径r范围内的领域点
	if (octree.radiusSearch(pn, L, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
	{
		for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
		{
			// 搜索点的坐标
			/*std::cout << "    " << cloud->points[pointIdxRadiusSearch[i]].x
			<< " " << cloud->points[pointIdxRadiusSearch[i]].y
			<< " " << cloud->points[pointIdxRadiusSearch[i]].z
			// 下标点与搜索点的平方距离
			<< " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;*/
			float distance = sqrt(pointRadiusSquaredDistance[i]);
			pcl::PointXYZ pn1(cloud->points[pointIdxRadiusSearch[i]].x, cloud->points[pointIdxRadiusSearch[i]].y, cloud->points[pointIdxRadiusSearch[i]].z);
			//_vectorPointNode.push_back(pn1);
			_vectorPointNode.insert(std::pair<float, pcl::PointXYZ>(distance, pn1));
		}
	}
	return _vectorPointNode;
}

// 设置点云和八叉树
void CCloudOctree::SetCloudOctree()
{

}
