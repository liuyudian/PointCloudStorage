#include "PointCloudManage.h"
#include <QtWidgets/QApplication>
#include"FileOption.h"
using namespace std;
int main(int argc, char *argv[])
{
	FileOption f;
	f.ReadAscFile("bunny.asc");
	QApplication a(argc, argv);
	PointCloudManage w;
	w.show();
	return a.exec();
}
