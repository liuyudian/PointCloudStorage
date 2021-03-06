#include "PointCloudManage.h"
#include <pcl/octree/octree.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
#include <ctime>
#define _USE_MATH_DEFINES
#include <cmath>
#include <algorithm>//注意要包含该头文件
#include <vector>
#include "ARGS.h"
using namespace std;
FileOption fo;

vector<MyPoint>vecPoint;
int countLeaf = 0;
void GetLeaf(pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud);
pcl::PolygonMesh triangles;//创建多边形网格，用于存储结果
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTriangles(new pcl::PointCloud<pcl::PointXYZ>);

// 存储极值的类


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
	connect(ui->pushButton_4, SIGNAL(clicked()), this, SLOT(GetLeafShow()));//另存为按钮
	connect(ui->pushButton_5, SIGNAL(clicked()), this, SLOT(Triangulation()));//三角网格剖分
	connect(ui->pushButtonGrid, SIGNAL(clicked()), this, SLOT(MeshGeneration()));//三角网格剖分																		  //connect(action11, SIGNAL(clicked()), this, SLOT(Triangulation()));
	connect(ui->pushButton_3, SIGNAL(clicked()), this, SLOT(Gridding()));//种子三角面片获取
}

// 自定义网格剖分
void PointCloudManage::MeshGeneration()
{
	// 点云数据读取
	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud1(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::io::loadPCDFile<pcl::PointXYZ>("bunny.pcd", *cloud1);

	if (pcl::io::loadPCDFile<pcl::PointXYZ>("bunny.pcd", *cloud1) == -1)
	{
		std::cout << "Cloud reading failed。" << std::endl;
		return;
	}
}
//种子三角面片获取
void PointCloudManage::Gridding()
{
	ARGS a;
	a.ArgsAlgorithm();
	a.Saveasply();
}


class MaxAndMin
{
public:
	Eigen::Vector3f min;
	Eigen::Vector3f max;
	MaxAndMin(Eigen::Vector3f min, Eigen::Vector3f max)
	{
		this->max = max;
		this->min = min;
	}
	bool isEqual(Eigen::Vector3f min, Eigen::Vector3f max)
	{
		if (min.x() == this->min.x()&&min.y() == this->min.y() &&min.z() == this->min.z() &&

			max.x() == this->max.x() &&max.y() == this->max.y() &&max.z() ==this-> max.z())
		{
			return true;
		}
		else
			return false;
	}
	bool operator<(const MaxAndMin& b)const
	{
		//return false;
		return (min.x() < b.min.x()) || 
			(min.x() == b.min.x()) && (min.y() <b.min.y()) || (min.x() == b.min.x() && min.y() == b.min.y() && min.z() < b.min.z());
		//return true;
		// 相等比较 
	}
	MaxAndMin()
	{}
};
bool cmp1(const MaxAndMin& a, const MaxAndMin&b)
{
	int x1 = a.min.x();
	int y1 = a.min.y();
	int z1 = a.min.z();

	int x2= b.min.x();
	int y2 = b.min.y();
	int z2 = b.min.z();
	
		if (x1 < x2)
		{
			return true;
		}
		else if (x1 == x2)
		{
			if (y1 <y2)
			{
				return true;
			}
			else if (y1 == y2)
			{
				if (z1 < z2)
				{
					return true;
				}
			}
		}
		return false;
}

// 叶子节点显示
void PointCloudManage::GetLeafShow()
{
	// 点云数据的存储

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud1(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::io::loadPCDFile<pcl::PointXYZ>("bunny.pcd", *cloud1);

	if (pcl::io::loadPCDFile<pcl::PointXYZ>("bunny.pcd", *cloud1) == -1)
	{
		std::cout << "Cloud reading failed。" << std::endl;
		return;
	}
	// 存储体素的下标
	map<int, vector<Eigen::Vector3f>>_bodyVoxel;

	// 存储叶子节点
	//pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudLeaf(new pcl::PointCloud<pcl::PointXYZ>);

	// 最小体素的边长
	float resolution = 5.0f;

	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);

	//octree.setTreeDepth(8);
	octree.setInputCloud(cloud1);
	// 从输入点云构建八叉树

	octree.addPointsFromInputCloud();

	
	pcl::PointXYZ searchPoint;

	int depth = octree.getTreeDepth();
	int countVoxel = 0;
	// 求出体素边界

	int idString = 0;
	for (auto it = octree.begin(depth);it != octree.end();++it)
	{
		if (it.isLeafNode())
		{
			Eigen::Vector3f  voxel_min, voxel_max;
			octree.getVoxelBounds(it, voxel_min, voxel_max);
			vector<Eigen::Vector3f> list;
			list.push_back(voxel_min);
			list.push_back(voxel_max);
			_bodyVoxel.insert(pair<int, vector<Eigen::Vector3f>>(countVoxel, list));
			std::cout << "最小值： " << voxel_min.x() << " " << voxel_min.y() << " " << voxel_min.z() << std::endl;
			std::cout << "最大值： " << voxel_max.x() << " " << voxel_max.y() << " " << voxel_max.z() << std::endl;
			std::cout << std::endl;
			countVoxel++;
		}
	}
	std::cout << "深度: " << depth << std::endl;

	// 画图
	vector<Eigen::Vector3f> list;
	Eigen::Vector3f  voxel_min, voxel_max;
	countLeaf = 0;
	for (auto it = _bodyVoxel.begin();it != _bodyVoxel.end();it++)
	{
		list = it->second;
		voxel_min = list[0];
		voxel_max = list[1];
		viewer->addCube(voxel_min.x(), voxel_max.x(), voxel_min.y(), voxel_max.y(), voxel_min.z(), voxel_max.z(), 1, 0, 0, std::to_string(countLeaf));
		countLeaf++;
	}
	std::cout << "**********叶子节点个数********" << octree.getLeafCount() << std::endl;
	std::cout << "叶子节点个数:  " << countLeaf << std::endl;
	// 对叶子节点容器初始化
}

// 按钮响应事件测试,打开文件
void PointCloudManage::ShowModel()
{
	QFile file;
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
	//ui->pushButton3->setText(tr("()"));

	// 数据读取
	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::io::loadPCDFile<pcl::PointXYZ>(s, *cloud);

	if (pcl::io::loadPCDFile<pcl::PointXYZ>(s, *cloud) == -1)
	{
		std::cout << "Cloud reading failed。" << std::endl;
		return;
	}

	//pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(cloud,"z"); // 按照z字段进行渲染
	//viewer->addPointCloud<pcl::PointXYZ>(cloud, fildColor, "sample cloud");

	// 设置单一颜色
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> singleColor(cloud, 0, 255, 0);//0-255  设置成绿色
	// 颜色显示
	viewer->addPointCloud<pcl::PointXYZ>(cloud, singleColor, "sample");//显示点云，其中fildColor为颜色显示
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

	std::cout <<"宽："<< cloud->width << std::endl;
	std::cout <<"高："<< cloud->height<<std::endl;
	viewer->updatePointCloud(cloud, "cloud");
}

//三角网格剖分
void PointCloudManage::Triangulation()
{
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("bunny.pcd", *cloudTriangles) == -1)
	{
		PCL_ERROR("Couldn't read file bunny.pcd\n");
			return ;
	}
	//Normal 法向量
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree < pcl::PointXYZ >::Ptr tree(new pcl::search::KdTree < pcl::PointXYZ >);
	tree->setInputCloud(cloudTriangles);
	n.setInputCloud(cloudTriangles);
	n.setSearchMethod(tree);
	n.setKSearch(20);
	n.compute(*normals);//法线
	//将点云和法线放在一起
	pcl::PointCloud<pcl::PointNormal>::Ptr Cloud_With_Normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloudTriangles, *normals, *Cloud_With_Normals);
	//创建查找树
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree < pcl::PointNormal >);
	tree2->setInputCloud(Cloud_With_Normals);
	//初始化对象
	pcl::GreedyProjectionTriangulation < pcl::PointNormal > gp3;
	//设置参数
	gp3.setSearchRadius(2.5);//设置连接点之间的最大距离（最大边长）用于确定k近邻的球半径，搜索半径
	gp3.setMu(2);//设置最近邻距离的乘子，已经得到每个点的最终搜索半径，即搜索半径
	gp3.setMaximumNearestNeighbors(100);//设置搜索的最近邻点的最大数量
	gp3.setMaximumSurfaceAngle(M_PI / 4);//45度最大平面角
	gp3.setMinimumAngle(M_PI / 18);//三角形的最小角度10度
	gp3.setMaximumAngle(2 * M_PI /2 );//三角形的最大角度120度
	gp3.setNormalConsistency(false);//若法向量一致，设为true
	//设置搜索方法和输出点云
	gp3.setInputCloud(Cloud_With_Normals);
	gp3.setSearchMethod(tree2);
	//执行重构，结果保存在triangles中
	gp3.reconstruct(triangles);
	std::cout << "************" << triangles.cloud.width << std::endl;

	std::cout<<"************"<<triangles.polygons.size()<<std::endl;
	//保存网格图
	pcl::io::saveVTKFile("bunny.vtk", triangles);//保存为vtk文件
	//增加顶点信息
	std::vector<int> parts = gp3.getPartIDs();
	std::vector<int> states = gp3.getPointStates();
	//cout << parts.size << endl;
	//显示结果图
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0,0,0);
	viewer->addPolygonMesh(triangles, "my");//设置显示的网格
	viewer->initCameraParameters();
	
	while (!viewer->wasStopped()) 
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	// vtk显示
	viewer->updatePointCloud(cloudTriangles, "cloud");

	//ui->qvtkWidget->update();
}

//把最终的模型另存为ply文件
void PointCloudManage::SaveAsPLY() 
{
	fo.SaveAsPLY(cloudTriangles,triangles);
}


