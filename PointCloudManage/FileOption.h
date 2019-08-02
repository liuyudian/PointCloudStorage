#pragma once

/*

	点云的文件操作

*/
class FileOption
{
public:

	//特性

	FileOption();
	~FileOption();

	//功能函数

	void ReadAscFile(); //读取.asc文件
	void AscToPcd(); //.asc文件转Pcd文件
	void SaveAsPLY(); //把划分好的三角面片另存为.ply文件
};

