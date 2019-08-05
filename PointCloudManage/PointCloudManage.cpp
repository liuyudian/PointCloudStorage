#include "PointCloudManage.h"
#include <pcl/octree/octree.h>
#include <iostream>
#include <vector>
#include <ctime>
using namespace std;
FileOption fo;

vector<MyPoint>vecPoint;
int countLeaf = 0;
void GetLeaf(pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud);

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
	//connect(ui->pushButton_3, SIGNAL(clicked()), this, SLOT(subdivision()));//���Ϊ��ť
	connect(ui->pushButton_4, SIGNAL(clicked()), this, SLOT(GetLeafShow()));//���Ϊ��ť

}

// Ҷ�ӽڵ���ʾ
void PointCloudManage::GetLeafShow()
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloudLeaf(new pcl::PointCloud<pcl::PointXYZRGB>);

	cloudLeaf->width = countLeaf;
	cloudLeaf->height = 1;
	cloudLeaf->points.resize(cloudLeaf->width * cloudLeaf->height);

	for (size_t i = 0; i < vecPoint.size(); ++i)
	{
		cloudLeaf->points[i].x = vecPoint[i].x;
		cloudLeaf->points[i].y = vecPoint[i].y;
		cloudLeaf->points[i].z = vecPoint[i].z;
		cloudLeaf->points[i].r = 255;
		cloudLeaf->points[i].g= 255;

		cloudLeaf->points[i].b = 255;

	}

	//subdivision();
	//system("pause");
	//subdivision();
	//pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZRGB> fildColor(cloudLeaf, "z"); // ����z�ֶν�����Ⱦ
	//viewer->addPointCloud<pcl::PointXYZRGB>(cloudLeaf, fildColor, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "sample cloud");

	viewer->updatePointCloud(cloudLeaf, "cloudLeaf");
	ui->qvtkWidget->update();
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

	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(cloud, "z"); // ����z�ֶν�����Ⱦ
	viewer->addPointCloud<pcl::PointXYZ>(cloud, fildColor, "sample cloud");
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
	// �洢Ҷ�ӽڵ�
	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudLeaf(new pcl::PointCloud<pcl::PointXYZ>);

	// ��С���صı߳�
	float resolution = 1.0;

	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);

	pcl::octree::OctreePointCloud<pcl::PointXYZ> octreenode(resolution);
//	octreenode.setTreeDepth(3);

	// �����������
	octreenode.setInputCloud(cloud);
	octree.setInputCloud(cloud);
	// ��������ƹ����˲���
	octreenode.addPointsFromInputCloud();

	octree.addPointsFromInputCloud();


	pcl::PointXYZ searchPoint;

	// ���Ҷ�ӽڵ�

	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		if (octreenode.findLeaf(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z))
		{
			// ��ʾҶ�ӽڵ�
			std::cout << "Ҷ�ӽڵ㣺 " << cloud->points[i].x
				<< " " << cloud->points[i].y
				<< " " << cloud->points[i].z << std::endl;
			MyPoint mypoint(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
			vecPoint.push_back(mypoint);
			countLeaf++;
			// �洢cloudLeaf
			//cloudLeaf->points[countLeaf] = cloud->points[i];
			//countLeaf++;
		}
	}
	std::cout << "**********Ҷ�ӽڵ����*********" << octree.getLeafCount() << std::endl;
	std::cout << "Ҷ�ӽڵ����:  " << countLeaf << std::endl;
	// ��Ҷ�ӽڵ�������ʼ��

	// ָ���뾶���ڼ�����
	float radius = 10;

	// ��ڵ��r�����ڵĵ�
	int index = 0;
	auto it=fo._mapPoint.find(index);

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

//�����յ�ģ�����Ϊply�ļ�
void PointCloudManage::SaveAsPLY() 
{

}


