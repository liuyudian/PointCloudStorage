#include "PointCloudManage.h"
#include <pcl/octree/octree.h>
#include <iostream>
#include <vector>
#include <ctime>
using namespace std;
void subdivision();
PointCloudManage::PointCloudManage(QWidget *parent):
	 QMainWindow(parent)
{
	ui->setupUi(this);
	//this->setWindowTitle("PCL viewer");
	// Set up the QVTK window
	viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
	ui->qvtkWidget->SetRenderWindow(viewer->getRenderWindow());
	viewer->setupInteractor(ui->qvtkWidget->GetInteractor(), ui->qvtkWidget->GetRenderWindow());
	ui->qvtkWidget->update();
	// 添加关联
	connect(ui->pushButton, SIGNAL(clicked()), this, SLOT(ShowModel()));//打开按钮
	//connect(ui->pushButton3, SIGNAL(clicked()), this, SLOT(VTK_Show()));
	connect(ui->pushButton_2, SIGNAL(clicked()), this, SLOT(SaveAsPLY()));//另存为按钮
	connect(ui->pushButton_3, SIGNAL(clicked()), this, SLOT(subdivision()));//另存为按钮

}

// 按钮响应事件测试,打开文件
void PointCloudManage::ShowModel()
{
	QFile file;
	FileOption fo;
	QString f = QFileDialog::getOpenFileName(this, QString("OpenFile"),
		QString("/"), QString("ASC(*.asc);;PCD(*.pcd)"));
	//QString转char * 
	QByteArray temp = f.toLocal8Bit();
	char *name = temp.data();
	qDebug() << f;
	fo.ReadAscFile(name);
	string s=fo.AscToPcd();
	VTK_Show(s);

	
	
	
}

//在vtk控件中显示点云
void PointCloudManage::VTK_Show(string s)
{
	
	ui->pushButton3->setText(tr("()"));
	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ>(s, *cloud);

	if (pcl::io::loadPCDFile<pcl::PointXYZ>(s, *cloud) == -1)
	{
		std::cout << "Cloud reading failed。" << std::endl;
		return;
	}

	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(cloud, "z"); // 按照z字段进行渲染
	viewer->addPointCloud<pcl::PointXYZ>(cloud, fildColor, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");

	std::cout << cloud->width << std::endl;
	std::cout << cloud->height;

	viewer->updatePointCloud(cloud, "cloud");
	ui->qvtkWidget->update();

	//system("pause");
	subdivision();
	
	
}

// 空间分割
void PointCloudManage::subdivision()
{
	std::cout << "*************************************空间剖分******************************" << std::endl;
	srand((unsigned int)time(NULL));

	// 定义和实例化一个PointCloud在这个数据结构，并随机生成点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	// Generate pointcloud data
	cloud->width = 1000;
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);

	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		cloud->points[i].x = 1024.0f * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].y = 1024.0f * rand() / (RAND_MAX + 1.0f);
		cloud->points[i].z = 1024.0f * rand() / (RAND_MAX + 1.0f);
	}

	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(cloud, "z"); // 按照z字段进行渲染
	viewer->addPointCloud<pcl::PointXYZ>(cloud, fildColor, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");

	viewer->updatePointCloud(cloud, "cloud");
	ui->qvtkWidget->update();


	// 设置分辨率并初始化octree实例，octree保持了叶子节点的下标。

	float resolution = 128.0f;

	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);

	pcl::octree::OctreePointCloud<pcl::PointXYZ> octreenode(resolution);

	// 设置输入点云
	octreenode.setInputCloud(cloud);
	octree.setInputCloud(cloud);
	// 从输入点云构建八叉树
	octreenode.addPointsFromInputCloud();

	octree.addPointsFromInputCloud();


	pcl::PointXYZ searchPoint;

	// 输出叶子节点
	std::cout << "叶子节点:  " << octreenode.getLeafCount() << std::endl;

	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		if (octreenode.findLeaf(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z))
		{
			std::cout << "叶子节点： " << cloud->points[i].x
				<< " " << cloud->points[i].y
				<< " " << cloud->points[i].z << std::endl;
		}
	}


	// 随机搜索点的生成
	searchPoint.x = 1024.0f * rand() / (RAND_MAX + 1.0f);
	searchPoint.y = 1024.0f * rand() / (RAND_MAX + 1.0f);
	searchPoint.z = 1024.0f * rand() / (RAND_MAX + 1.0f);

	// 按照半径进行搜索
	// Neighbors within radius search

	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;

	// 指定半径
	float radius = 256.0f * rand() / (RAND_MAX + 1.0f);

	std::cout << " Neighbors within radius search at " << searchPoint.x
		<< " " << searchPoint.y
		<< " " << searchPoint.z
		<< ") with radius=" << radius << std::endl;

    // 随机生成的某个顶点在空间半径r范围内的领域点
	if (octree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
	{
		for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
			// 搜索点的坐标
			std::cout << "    " << cloud->points[pointIdxRadiusSearch[i]].x
			<< " " << cloud->points[pointIdxRadiusSearch[i]].y
			<< " " << cloud->points[pointIdxRadiusSearch[i]].z
			// 下标点与搜索点的平方距离
			<< " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
	}

}

//把最终的模型另存为ply文件
void PointCloudManage::SaveAsPLY() 
{

}


