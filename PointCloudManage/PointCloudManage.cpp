#include "PointCloudManage.h"

PointCloudManage::PointCloudManage(QWidget *parent):
	 QMainWindow(parent)
{
	ui.setupUi(this);
	connect(ui.pushButton, SIGNAL(clicked()), this, SLOT(OpenFile()));


}

// ��ť��Ӧ�¼�����,���ļ�
void PointCloudManage::OpenFile()
{
	QFile file;
	QString f = QFileDialog::getOpenFileName(this, QString("OpenFile"),
		QString("/"), QString("ASC(*.asc)"));
	qDebug() << f;
	VTK_Show(f);

	
	
	
}

//��vtk�ؼ�����ʾ����
void PointCloudManage::VTK_Show(QString s) 
{
	ui.pushButton->setText(tr("(hello)"));
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::io::loadPCDFile<pcl::PointXYZRGB>("bun0.pcd", *cloud);

	if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("bun0.pcd", *cloud) == -1)
	{
		std::cout << "Cloud reading failed." << std::endl;
		return;
	}

	std::cout << cloud->width << std::endl;
	std::cout << cloud->height;

	viewer->updatePointCloud(cloud, "cloud");
	ui.qvtkWidget->update();
	system("pause");
}