#include "PointCloudManage.h"
#include <QtWidgets/QApplication>
using namespace std;
int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	PointCloudManage w;
	w.show();
	return a.exec();
}
