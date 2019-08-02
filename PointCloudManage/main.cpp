#include "PointCloudManage.h"
#include <QtWidgets/QApplication>
#include"FileOption.h"
using namespace std;
void viewerOneOff(pcl::visualization::PCLVisualizer& viewer) {
	viewer.setBackgroundColor(1.0f, 0.5f, 1.0f);
}
int main(int argc, char *argv[])
{
	FileOption f;
	//f.ReadAscFile("bunny.asc");
	//f.AscToPcd();
	//f.ReadPcd();
	QApplication a(argc, argv);
	PointCloudManage w;
	w.show();
	return a.exec();
}
