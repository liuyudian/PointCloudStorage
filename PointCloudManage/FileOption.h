#pragma once

/*

	���Ƶ��ļ�����

*/
class FileOption
{
public:

	//����

	FileOption();
	~FileOption();

	//���ܺ���

	void ReadAscFile(); //��ȡ.asc�ļ�
	void AscToPcd(); //.asc�ļ�תPcd�ļ�
	void SaveAsPLY(); //�ѻ��ֺõ�������Ƭ���Ϊ.ply�ļ�
};

