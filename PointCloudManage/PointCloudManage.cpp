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
#include <algorithm>//ע��Ҫ������ͷ�ļ�
#include <vector>
#include "ARGS.h"
using namespace std;
FileOption fo;

vector<MyPoint>vecPoint;
int countLeaf = 0;
void GetLeaf(pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud);
pcl::PolygonMesh triangles;//����������������ڴ洢���
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTriangles(new pcl::PointCloud<pcl::PointXYZ>);

// �洢��ֵ����


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
	// ��ӹ���
	connect(ui->pushButton, SIGNAL(clicked()), this, SLOT(ShowModel()));//�򿪰�ť
	//connect(ui->pushButton3, SIGNAL(clicked()), this, SLOT(VTK_Show()));
	connect(ui->pushButton_2, SIGNAL(clicked()), this, SLOT(SaveAsPLY()));//���Ϊ��ť
	connect(ui->pushButton_4, SIGNAL(clicked()), this, SLOT(GetLeafShow()));//���Ϊ��ť
	connect(ui->pushButton_5, SIGNAL(clicked()), this, SLOT(Triangulation()));//���������ʷ�
	connect(ui->pushButtonGrid, SIGNAL(clicked()), this, SLOT(MeshGeneration()));//���������ʷ�																		  //connect(action11, SIGNAL(clicked()), this, SLOT(Triangulation()));
	connect(ui->pushButton_3, SIGNAL(clicked()), this, SLOT(Getzhongzi()));//����������Ƭ��ȡ
}

// �Զ��������ʷ�
void PointCloudManage::MeshGeneration()
{
	// �������ݶ�ȡ
	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud1(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::io::loadPCDFile<pcl::PointXYZ>("bunny.pcd", *cloud1);

	if (pcl::io::loadPCDFile<pcl::PointXYZ>("bunny.pcd", *cloud1) == -1)
	{
		std::cout << "Cloud reading failed��" << std::endl;
		return;
	}
}
//����������Ƭ��ȡ
void PointCloudManage::Getzhongzi()
{
	ARGS a;
	a.GetARGS();
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
		// ��ȱȽ� 
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

// Ҷ�ӽڵ���ʾ
void PointCloudManage::GetLeafShow()
{
	// �������ݵĴ洢

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud1(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::io::loadPCDFile<pcl::PointXYZ>("bunny1.pcd", *cloud1);

	if (pcl::io::loadPCDFile<pcl::PointXYZ>("bunny1.pcd", *cloud1) == -1)
	{
		std::cout << "Cloud reading failed��" << std::endl;
		return;
	}
	// �洢���ص��±�
	map<int, vector<Eigen::Vector3f>>_bodyVoxel;

	// �洢Ҷ�ӽڵ�
	//pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudLeaf(new pcl::PointCloud<pcl::PointXYZ>);

	// ��С���صı߳�
	float resolution = 0.015f;

	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);

	//octree.setTreeDepth(8);
	octree.setInputCloud(cloud1);
	// ��������ƹ����˲���

	octree.addPointsFromInputCloud();

	
	pcl::PointXYZ searchPoint;

	int depth = octree.getTreeDepth();
	int countVoxel = 0;
	// ������ر߽�

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
			std::cout << "��Сֵ�� " << voxel_min.x() << " " << voxel_min.y() << " " << voxel_min.z() << std::endl;
			std::cout << "���ֵ�� " << voxel_max.x() << " " << voxel_max.y() << " " << voxel_max.z() << std::endl;
			std::cout << std::endl;
			countVoxel++;
		}
	}
	// ��ͼ
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
	std::cout << "**********Ҷ�ӽڵ����********" << octree.getLeafCount() << std::endl;
	std::cout << "Ҷ�ӽڵ����:  " << countLeaf << std::endl;
	// ��Ҷ�ӽڵ�������ʼ��
}

// ��ť��Ӧ�¼�����,���ļ�
void PointCloudManage::ShowModel()
{
	QFile file;
	QString f = QFileDialog::getOpenFileName(this, QString("OpenFile"),
		QString("/"), QString("ASC(*.asc);;PCD(*.pcd)"));
	//QStringתchar * 
	QByteArray temp = f.toLocal8Bit();
	char *name = temp.data();
	qDebug() << f;
	fo.ReadAscFile(name);
	string s=fo.AscToPcd();
	VTK_Show(s);
}

//��vtk�ؼ�����ʾ����
void PointCloudManage::VTK_Show(string s)
{
	ui->pushButton3->setText(tr("()"));

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::io::loadPCDFile<pcl::PointXYZ>(s, *cloud);

	if (pcl::io::loadPCDFile<pcl::PointXYZ>(s, *cloud) == -1)
	{
		std::cout << "Cloud reading failed��" << std::endl;
		return;
	}

	//pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(cloud,"z"); // ����z�ֶν�����Ⱦ
	//viewer->addPointCloud<pcl::PointXYZ>(cloud, fildColor, "sample cloud");

	// ���õ�һ��ɫ
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> singleColor(cloud, 0, 255, 0);//0-255  ���ó���ɫ
	// ��ɫ��ʾ
	viewer->addPointCloud<pcl::PointXYZ>(cloud, singleColor, "sample");//��ʾ���ƣ�����fildColorΪ��ɫ��ʾ
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

	std::cout <<"��"<< cloud->width << std::endl;
	std::cout <<"�ߣ�"<< cloud->height<<std::endl;
	viewer->updatePointCloud(cloud, "cloud");
}

//���������ʷ�
void PointCloudManage::Triangulation()
{
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("bunny.pcd", *cloudTriangles) == -1)
	{
		PCL_ERROR("Couldn't read file bunny.pcd\n");
			return ;
	}
	//Normal ������
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree < pcl::PointXYZ >::Ptr tree(new pcl::search::KdTree < pcl::PointXYZ >);
	tree->setInputCloud(cloudTriangles);
	n.setInputCloud(cloudTriangles);
	n.setSearchMethod(tree);
	n.setKSearch(20);
	n.compute(*normals);//����
	//�����ƺͷ��߷���һ��
	pcl::PointCloud<pcl::PointNormal>::Ptr Cloud_With_Normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloudTriangles, *normals, *Cloud_With_Normals);
	//����������
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree < pcl::PointNormal >);
	tree2->setInputCloud(Cloud_With_Normals);
	//��ʼ������
	pcl::GreedyProjectionTriangulation < pcl::PointNormal > gp3;
	//���ò���
	gp3.setSearchRadius(2.5);//�������ӵ�֮��������루���߳�������ȷ��k���ڵ���뾶�������뾶
	gp3.setMu(2);//��������ھ���ĳ��ӣ��Ѿ��õ�ÿ��������������뾶���������뾶
	gp3.setMaximumNearestNeighbors(100);//��������������ڵ���������
	gp3.setMaximumSurfaceAngle(M_PI / 4);//45�����ƽ���
	gp3.setMinimumAngle(M_PI / 18);//�����ε���С�Ƕ�10��
	gp3.setMaximumAngle(2 * M_PI /2 );//�����ε����Ƕ�120��
	gp3.setNormalConsistency(false);//��������һ�£���Ϊtrue
	//���������������������
	gp3.setInputCloud(Cloud_With_Normals);
	gp3.setSearchMethod(tree2);
	//ִ���ع������������triangles��
	gp3.reconstruct(triangles);
	std::cout << "************" << triangles.cloud.width << std::endl;

	std::cout<<"************"<<triangles.polygons.size()<<std::endl;
	//��������ͼ
	pcl::io::saveVTKFile("bunny.vtk", triangles);//����Ϊvtk�ļ�
	//���Ӷ�����Ϣ
	std::vector<int> parts = gp3.getPartIDs();
	std::vector<int> states = gp3.getPointStates();
	//cout << parts.size << endl;
	//��ʾ���ͼ
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0,0,0);
	viewer->addPolygonMesh(triangles, "my");//������ʾ������
	viewer->initCameraParameters();
	
	while (!viewer->wasStopped()) 
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	// vtk��ʾ
	viewer->updatePointCloud(cloudTriangles, "cloud");
	//ui->qvtkWidget->update();
}

//�����յ�ģ�����Ϊply�ļ�
void PointCloudManage::SaveAsPLY() 
{
	fo.SaveAsPLY(cloudTriangles,triangles);
}


