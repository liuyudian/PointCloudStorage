#include "PointCloudManage.h"
#include <pcl/octree/octree.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
#include <ctime>
#define _USE_MATH_DEFINES
#include <cmath>
#include <algorithm>//ע��Ҫ������ͷ�ļ�
#include <vector>
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
	//connect(action11, SIGNAL(clicked()), this, SLOT(Triangulation()));
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

	pcl::io::loadPCDFile<pcl::PointXYZ>("bunny.pcd", *cloud1);

	if (pcl::io::loadPCDFile<pcl::PointXYZ>("bunny.pcd", *cloud1) == -1)
	{
		std::cout << "Cloud reading failed��" << std::endl;
		return;
	}
	// �洢���ص��±�
	map<int, vector<Eigen::Vector3f>>_bodyVoxel;

	// �洢Ҷ�ӽڵ�
	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudLeaf(new pcl::PointCloud<pcl::PointXYZ>);

	// ��С���صı߳�
	float resolution = 128.0f;

	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);

	//octree.setTreeDepth(8);
	octree.setInputCloud(cloud1);
	// ��������ƹ����˲���

	octree.addPointsFromInputCloud();


	pcl::PointXYZ searchPoint;

	int depth = 5;
	int countVoxel = 0;
	// ������ر߽�
	for (auto it = octree.begin(depth);it != octree.end();++it)
	{
		Eigen::Vector3f  voxel_min, voxel_max;
		octree.getVoxelBounds(it, voxel_min, voxel_min);
		vector<Eigen::Vector3f> list;
		list.push_back(voxel_min);
		list.push_back(voxel_max);
		_bodyVoxel.insert(pair<int, vector<Eigen::Vector3f>>(countVoxel, list));
		std::cout <<"���Ϊ8�����ص���Сֵ�� "<<   voxel_min.x()<<" " << voxel_min.y()<<" " << voxel_min.z() <<std:: endl;
		std::cout << "���Ϊ8�����ص����ֵ�� " << voxel_max.x() << " " << voxel_max.y() << " " << voxel_max.z() <<std:: endl;
		std::cout << std::endl;
		countVoxel++;
	}
	pcl::visualization::PCLVisualizer viewer("3D Bunny Single Range rendering");
	viewer.setBackgroundColor(0, 1, 0);
	map<MaxAndMin, int>_mapMaxMin;
	// ��������
	std::cout << "ȥ��ǰ�ļ�ֵ������ " << countVoxel << std::endl;
	int countll=0;
	for (auto it = _bodyVoxel.begin();it != _bodyVoxel.end();it++)
	{
		vector<Eigen::Vector3f> list = it->second;
		MaxAndMin mam(list[0], list[1]);
		 auto itt=_mapMaxMin.insert(pair<MaxAndMin, int>(mam, 0));
		 if (itt.second)
		 {
			 std::cout << "���Ϊ8�����ص���Сֵ�� " << list[0].x() << " " << list[0].y() << " " << list[0].z() << std::endl;
			 //  addCube (float x_min, float x_max, float y_min, float y_max, float z_min, float z_max,
	         //  double r = 1.0, double g = 1.0, double b = 1.0, const std::strings&id = "cube", int viewport = 0);
			  viewer.addCube(list[1].x(), list[0].x(), list[1].y(), list[0].y(), list[1].z(), list[0].z(), 255, 0, 0, "cube"+ countll, 0);
			 countll++;
		 }
	}
	std::cout << "ȥ�غ�ļ�ֵ������ " << _mapMaxMin.size() << std::endl;
	while (!viewer.wasStopped())
	{
		viewer.spinOnce(1);
	}
	std::cout << "ȥ�غ�ļ�ֵ������ " << _mapMaxMin.size() << std::endl;
	// ����ȥ��
	//sort(_vector.begin(), _vector.end(),cmp1);
	//_vector.erase(unique(_vector.begin(), _vector.end()), _vector.end());

	// ��Ҷ�ӽڵ�������ʼ��

	// ָ���뾶���ڼ�����
	float radius = 10;

	// ��ڵ��r�����ڵĵ�
	int index = 0;
	auto it = fo._mapPoint.find(index);

	if (it != fo._mapPoint.end())
	{
		searchPoint.x = it->second.x;
		searchPoint.y = it->second.y;
		searchPoint.z = it->second.z;
	}

	std::cout << " Neighbors within radius search at " << searchPoint.x
		<< " " << searchPoint.y
		<< " " << searchPoint.z
		<< ") with radius=" << radius << std::endl;

	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;

	// ������ɵ�ĳ�������ڿռ�뾶r��Χ�ڵ������
	if (octree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
	{
		for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
			// �����������
			std::cout << "    " << cloud->points[pointIdxRadiusSearch[i]].x
			<< " " << cloud->points[pointIdxRadiusSearch[i]].y
			<< " " << cloud->points[pointIdxRadiusSearch[i]].z
			// �±�����������ƽ������
			<< " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
	}
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
	ui->qvtkWidget->update();
	// ����Ҷ�ӽڵ�
	 GetLeaf(cloud);
}

// ��ȡҶ�ӽڵ�
void GetLeaf(pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud)
{
	
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
	gp3.setSearchRadius(8);//�������ӵ�֮��������루���߳�������ȷ��k���ڵ���뾶�������뾶
	gp3.setMu(2.5);//��������ھ���ĳ��ӣ��Ѿ��õ�ÿ��������������뾶
	gp3.setMaximumNearestNeighbors(150);//��������������ڵ���������
	gp3.setMaximumSurfaceAngle(2*M_PI / 1);//45�����ƽ���
	gp3.setMinimumAngle(M_PI / 144);//�����ε���С�Ƕ�10��
	gp3.setMaximumAngle(2 * M_PI /2.2 );//�����ε����Ƕ�120��
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
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0,0,0);
	viewer->addPolygonMesh(triangles, "my");//������ʾ������
	viewer->initCameraParameters();
	while (!viewer->wasStopped()) 
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
	//viewer->updatePointCloud(cloudTriangles, "cloud");
	//ui->qvtkWidget->update();
}

//�����յ�ģ�����Ϊply�ļ�
void PointCloudManage::SaveAsPLY() 
{
	fo.SaveAsPLY(cloudTriangles,triangles);
}


