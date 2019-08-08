#pragma once
#include<vector>
using namespace std;


// 点结构
struct PointNode
{
	float x, y, z;

};
// 边结构
struct Edge
{
	PointNode startNode;
	PointNode endNode;

};
// 三角面片结构
struct Surface
{
	Edge len1;
	Edge len2;
	Edge len3;
};
class ARGS
{
public:

	// 特性

	vector<PointNode>candidatePointNode;


	ARGS();
	~ARGS();

	// 选择种子三角片
	Surface SelectSurface();


	// 获取候选点集
	vector<PointNode> GetCandidate(Surface seedSurface);

	// 候选点添加代价
	vector<PointNode> AddCost(Surface seedSurface);

	// 最佳点筛选
	vector<PointNode> GetBestPointNode(vector<PointNode> candidatePointNode);
};

