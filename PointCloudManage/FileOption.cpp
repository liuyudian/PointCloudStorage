#include "FileOption.h"
using namespace std;


FileOption::FileOption()
{
}


FileOption::~FileOption()
{
}


//��ȡ.asc�ļ�����
void FileOption::ReadAscFile(const char *cfilename)
{
	FILE *pfile;
	long  size;
	char *buffer;
	size_t result;
	//���ļ�
	fopen_s(&pfile, cfilename, "rb");
	if (pfile == NULL)
	{
		fputs("file error", stderr);
		exit(1);

	}
	//��ȡ�ļ���С
	fseek(pfile, 0, SEEK_END);
	size = ftell(pfile);

	rewind(pfile);
	//Ϊ�ļ������ڴ�
	buffer = (char*)malloc(sizeof(char)*size);
	if (buffer == NULL)
	{
		fputs("memory error", stderr);
		exit(2);
	}
	//���ļ�������buffer��
	result = fread(buffer, 1, size, pfile);
	if (result != size)
	{
		fputs("reading error", stderr);
		exit(3);
	}
	//�ر��ļ����ͷ��ڴ�
	fclose(pfile);
	cout << size << endl;
	ios::sync_with_stdio(false);
	AscToPcd(buffer);
	ios::sync_with_stdio(true);
	free(buffer);

}


//.asc�ļ�תPcd�ļ�
void FileOption::AscToPcd(const char *buffer)
{
	
}


//�ѻ��ֺõ�������Ƭ���Ϊ.ply�ļ�
void FileOption::SaveAsPLY() 
{

}

