#include "PointCloudMange.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
	cout << endl;
	QApplication a(argc, argv);
	PointCloudMange w;
	w.show();
	return a.exec();
}
