#include "PointCloudManage.h"
#include <iostream>
#include <QtWidgets/QApplication>
using namespace std;
int main(int argc, char *argv[])
{
	cout << "hello world! 123123" << endl;
	QApplication a(argc, argv);
	PointCloudManage w;
	w.show();
	return a.exec();
}
