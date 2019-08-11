#include "FileOption.h"
//#include "PointCloudManage.h"


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
	ofstream of(fileName.c_str());
	of.precision(std::numeric_limits<double>::digits10);
	//ofstream OpenFile("D:\\vs2017projects\\Task2\\testmap.ply");
	if (of.fail())
	{
		std::cout << "打开文件错误!" << std::endl;
	}
	of << "# .PCD v.5 - Point Cloud Data file format" << std::endl;
	of << "VERSION .5" << std::endl;
	of << "FIELDS x y z " << std::endl;
	of << "SIZE 4 4 4 " << std::endl;
	of << "TYPE F F F " << std::endl;
	of << "COUNT 1 1 1 " << endl;
	of << "WIDTH " << _mapPoint.size() << std::endl;
	of << "HEIGHT 1" << endl;
	of << "POINTS " << _mapPoint.size() << std::endl;
	of << "DATA ascii"<< std::endl;
	// 读取点的信息
	for (auto iter = _mapPoint.begin(); iter != _mapPoint.end(); iter++)
	{
		of << iter->second.x<< " " << iter->second.y << " " << iter->second.z <<" "<<iter->second.R<<" "<<iter->second.G<<" "<<iter->second.B<< std::endl;
	}
	return fileName;
}


//把划分好的三角面片另存为.ply文件
/*void FileOption::SaveAsPLY(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTriangles,pcl::PolygonMesh triangles)
{
	string fileName = "bunny.ply";
	ofstream of(fileName.c_str());
	of.precision(std::numeric_limits<double>::digits10);
	//ofstream OpenFile("D:\\vs2017projects\\Task2\\testmap.ply");
	if (of.fail())
	{
		std::cout << "打开文件错误!" << std::endl;
	}
	of << "ply" << endl;
	of << "format ascii 1.0" << endl;
	of << "comment generated by platoply" << endl;
	of << "element vertex " << triangles.cloud.width << endl;
	of << "property float32 x" << endl;
	of << "property float32 y" << endl;
	of << "property float32 z" << endl;
	of << "element face " << triangles.polygons.size() << endl;
	of << "property list uint8 int32 vertex_indices" << endl;
	of << "end_header" << endl;
	for (int it = 0;it < cloudTriangles->points.size();it++)
	{
		of << " " << cloudTriangles->points[it].x << " " << cloudTriangles->points[it].y << " " << cloudTriangles->points[it].z << endl;
	}

	for (int it = 0;it < triangles.polygons.size();it++)
	{
		of << 3 << " " << triangles.polygons[it].vertices[0] << " " << triangles.polygons[it].vertices[1] << " " << triangles.polygons[it].vertices[2] << endl;
	}
}*/
 

// 读取PCD文件
void FileOption::ReadPcd()
{
}

