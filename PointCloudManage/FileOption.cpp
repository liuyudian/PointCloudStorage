#include "FileOption.h"
using namespace std;


FileOption::FileOption()
{
}


FileOption::~FileOption()
{
}


//读取.asc文件数据
void FileOption::ReadAscFile(const char *cfilename)
{
	FILE *pfile;
	long  size;
	char *buffer;
	size_t result;
	//打开文件
	fopen_s(&pfile, cfilename, "rb");
	if (pfile == NULL)
	{
		fputs("file error", stderr);
		exit(1);

	}
	//获取文件大小
	fseek(pfile, 0, SEEK_END);
	size = ftell(pfile);

	rewind(pfile);
	//为文件分配内存
	buffer = (char*)malloc(sizeof(char)*size);
	if (buffer == NULL)
	{
		fputs("memory error", stderr);
		exit(2);
	}
	//将文件拷贝到buffer中
	result = fread(buffer, 1, size, pfile);
	if (result != size)
	{
		fputs("reading error", stderr);
		exit(3);
	}
	//关闭文件，释放内存
	fclose(pfile);
	cout << size << endl;
	ios::sync_with_stdio(false);
	AscToPcd(buffer);
	ios::sync_with_stdio(true);
	free(buffer);

}


//.asc文件转Pcd文件
void FileOption::AscToPcd(const char *buffer)
{
	
}


//把划分好的三角面片另存为.ply文件
void FileOption::SaveAsPLY() 
{

}

