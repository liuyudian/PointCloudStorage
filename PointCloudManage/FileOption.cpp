#include "FileOption.h"
//#include "PointCloudManage.h"
#include <map>
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
	ios::sync_with_stdio(false);
	//AscToPcd(buffer);
	//读取buffer中的点数据
	int i = 0;
	string us1, us2, us3;
	//float x, y, z; //点的坐标
	MyPoint a;
	stringstream ss(buffer);
	ss.get();
	do {
		ss >> a.x >> a.y >> a.z ;
		a.R = 1;
		a.G = 1;
		a.B = 1;
		if (a.x== NULL)
			break;

		_mapPoint.insert(std::pair<int, MyPoint>(i, a));
		//cout << a.x << " " << a.y << " " << a.z << endl;
		i++;
	} while (1);
	ios::sync_with_stdio(true);
	free(buffer);

}


//.asc文件转Pcd文件
string FileOption::AscToPcd()
{
	if (_mapPoint.size() == 0)
		return 0;
	string fileName = "bunny.pcd";
	ofstream OpenFile(fileName.c_str());
	OpenFile.precision(std::numeric_limits<double>::digits10);
	//ofstream OpenFile("D:\\vs2017projects\\Task2\\testmap.ply");
	if (OpenFile.fail())
	{
		cout << "打开文件错误!" << endl;
	}
	OpenFile << "# .PCD v.5 - Point Cloud Data file format" << endl;
	OpenFile << "VERSION .5" << endl;
	OpenFile << "FIELDS x y z " <<  endl;
	OpenFile << "SIZE 4 4 4 " << endl;
	OpenFile << "TYPE F F F " << endl;
	OpenFile << "COUNT 1 1 1 " << endl;
	OpenFile << "WIDTH " << _mapPoint.size() << endl;
	OpenFile << "HEIGHT 1" << endl;
	OpenFile << "POINTS " << _mapPoint.size() << endl;
	OpenFile << "DATA ascii"<< endl;
	// 读取点的信息
	for (auto iter = _mapPoint.begin(); iter != _mapPoint.end(); iter++)
	{
		OpenFile << iter->second.x<< " " << iter->second.y << " " << iter->second.z <<" "<<iter->second.R<<" "<<iter->second.G<<" "<<iter->second.B<< endl;
	}
	return fileName;
}


//把划分好的三角面片另存为.ply文件
void FileOption::SaveAsPLY(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTriangles,pcl::PolygonMesh triangles)
{
	string fileName = "bunny.ply";
	ofstream OpenFile(fileName.c_str());
	OpenFile.precision(std::numeric_limits<double>::digits10);
	//ofstream OpenFile("D:\\vs2017projects\\Task2\\testmap.ply");
	if (OpenFile.fail())
	{
		std::cout << "打开文件错误!" << std::endl;
	}
	OpenFile << "ply" << endl;
	OpenFile << "format ascii 1.0" << endl;
	OpenFile << "comment generated by platoply" << endl;
	OpenFile << "element vertex " << triangles.cloud.width << endl;
	OpenFile << "property float32 x" << endl;
	OpenFile << "property float32 y" << endl;
	OpenFile << "property float32 z" << endl;
	OpenFile << "element face " << triangles.polygons.size() << endl;
	OpenFile << "property list uint8 int32 vertex_indices" << endl;
	OpenFile << "end_header" << endl;
	for (int it = 0;it < cloudTriangles->points.size();it++)
	{
		OpenFile << " " << cloudTriangles->points[it].x << " " << cloudTriangles->points[it].y << " " << cloudTriangles->points[it].z << endl;
	}

	for (int it = 0;it < triangles.polygons.size();it++)
	{
		OpenFile << 3 << " " << triangles.polygons[it].vertices[0] << " " << triangles.polygons[it].vertices[1] << " " << triangles.polygons[it].vertices[2] << endl;
	}
}
 

// 读取PCD文件
void FileOption::ReadPcd()
{
}

