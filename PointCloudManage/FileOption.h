#pragma once
//����ͷ�ļ�
#include <fstream>
#include <sstream>
#include <iostream>
#include <map>
#include"MyPoint.h"
using namespace std;

/*

	���Ƶ��ļ�����

*/
class FileOption
{
public:

	//����
	map<int, MyPoint> _mapPoint;
	FileOption();
	~FileOption();

	//���ܺ���

	void ReadAscFile(const char *cfilename); //��ȡ.asc�ļ�
	void AscToPcd(); //.asc�ļ�תPcd�ļ�
	void SaveAsPLY(); //�ѻ��ֺõ�������Ƭ���Ϊ.ply�ļ�
	void ReadPcd(); // ��ȡPCD�ļ�
	
};

