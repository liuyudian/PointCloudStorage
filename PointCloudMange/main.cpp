#include "PointCloudMange.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
	cout << "hello world 123"<<endl;
	QApplication a(argc, argv);
	PointCloudMange w;
	w.show();
	return a.exec();
}
