#include "PointCloudManage.h"
#include <pcl/octree/octree.h>
#include <iostream>
#include <vector>
#include <ctime>
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
	// ��ӹ���
	connect(ui->pushButton, SIGNAL(clicked()), this, SLOT(ShowModel()));
	connect(ui->pushButton3, SIGNAL(clicked()), this, SLOT(VTK_Show()));
	

}

// ��ť��Ӧ�¼�����,���ļ�
void PointCloudManage::ShowModel()
{
	QFile file;
	QString f = QFileDialog::getOpenFileName(this, QString("OpenFile"),
		QString("/"), QString("ASC(*.asc)"));
	qDebug() << f;
	//VTK_Show(f);

	
	
	
}

//��vtk�ؼ�����ʾ����
void PointCloudManage::VTK_Show()
{
	ui->pushButton3->setText(tr("(hello)"));
	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile<pcl::PointXYZ>("bunny.pcd", *cloud);

	if (pcl::io::loadPCDFile<pcl::PointXYZ>("bunny.pcd", *cloud) == -1)
	{
		std::cout << "Cloud reading failed." << std::endl;
		return;
	}

	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(cloud, "z"); // ����z�ֶν�����Ⱦ
	viewer->addPointCloud<pcl::PointXYZ>(cloud, fildColor, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");

	std::cout << cloud->width << std::endl;
	std::cout << cloud->height;

	viewer->updatePointCloud(cloud, "cloud");
	ui->qvtkWidget->update();

	//system("pause");
	subdivision();
	
	
}

// �ռ�ָ�
void subdivision()
{
	cout << "*************************************�ռ��ʷ�******************************" << endl;
	srand((unsigned int)time(NULL));

	// �����ʵ����һ��PointCloud��������ݽṹ����������ɵ���
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

	// ���÷ֱ��ʲ���ʼ��octreeʵ����octree������Ҷ�ӽڵ���±ꡣ

	float resolution = 128.0f;

	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);

	pcl::octree::OctreePointCloud<pcl::PointXYZ> octreenode(resolution);

	// �����������
	octreenode.setInputCloud(cloud);
	octree.setInputCloud(cloud);
	// ��������ƹ����˲���
	octreenode.addPointsFromInputCloud();

	octree.addPointsFromInputCloud();


	pcl::PointXYZ searchPoint;

	// ���Ҷ�ӽڵ�
	cout << "Ҷ�ӽڵ�:  " << octreenode.getLeafCount() << endl;

	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		if (octreenode.findLeaf(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z))
		{
			std::cout << "Ҷ�ӽڵ㣺 " << cloud->points[i].x
				<< " " << cloud->points[i].y
				<< " " << cloud->points[i].z << endl;
		}
	}


	// ��������������
	searchPoint.x = 1024.0f * rand() / (RAND_MAX + 1.0f);
	searchPoint.y = 1024.0f * rand() / (RAND_MAX + 1.0f);
	searchPoint.z = 1024.0f * rand() / (RAND_MAX + 1.0f);

	// ���հ뾶��������
	// Neighbors within radius search

	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;

	// ָ���뾶
	float radius = 256.0f * rand() / (RAND_MAX + 1.0f);

	std::cout << "Neighbors within radius search at (" << searchPoint.x
		<< " " << searchPoint.y
		<< " " << searchPoint.z
		<< ") with radius=" << radius << std::endl;

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


