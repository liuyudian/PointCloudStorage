#include "PointCloudManage.h"

PointCloudManage::PointCloudManage(QWidget *parent):
	 QMainWindow(parent)
{
	ui.setupUi(this);
	connect(ui.pushButton, SIGNAL(clicked()), this, SLOT(OpenFile()));


}

// 按钮响应事件测试,打开文件
void PointCloudManage::OpenFile()
{
	ui.pushButton->setText(tr("(hello)"));
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::io::loadPCDFile<pcl::PointXYZRGB>("bun0.pcd", *cloud);

	if (pcl::io::loadPCDFile<pcl::PointXYZRGB>("bun0.pcd", *cloud) == -1)
	{
		std::cout << "Cloud reading failed." << std::endl;
		return ;
	}

	std::cout << cloud->width << std::endl;
	std::cout << cloud->height;

	viewer->updatePointCloud(cloud, "cloud");
	ui.qvtkWidget->update();
	system("pause");
}