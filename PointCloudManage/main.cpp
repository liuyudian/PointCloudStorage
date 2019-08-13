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
	//ui.pushButton->setText(tr("(hello)"));
	//CCloudOctree cld;
	//float L = 0;
	//pcl::PointXYZ pn;
	//cld.GetField( L,  pn);
	QApplication a(argc, argv);
	PointCloudManage w;
	w.show();
	return a.exec();
}

