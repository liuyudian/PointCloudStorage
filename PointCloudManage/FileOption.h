#pragma once
//导入头文件
#include <fstream>
#include <sstream>
#include <iostream>
#include <map>
#include"MyPoint.h"
using namespace std;

/*

	点云的文件操作

*/
class FileOption
{
public:

	//特性
	map<int, MyPoint> _mapPoint;
	FileOption();
	~FileOption();

	//功能函数

	void ReadAscFile(const char *cfilename); //读取.asc文件
	void AscToPcd(); //.asc文件转Pcd文件
	void SaveAsPLY(); //把划分好的三角面片另存为.ply文件
	void ReadPcd(); // 读取PCD文件
	
};

