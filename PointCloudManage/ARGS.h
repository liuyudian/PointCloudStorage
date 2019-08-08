#pragma once
#include<vector>
using namespace std;


// ��ṹ
struct PointNode
{
	float x, y, z;

};
// �߽ṹ
struct Edge
{
	PointNode startNode;
	PointNode endNode;

};
// ������Ƭ�ṹ
struct Surface
{
	Edge len1;
	Edge len2;
	Edge len3;
};
class ARGS
{
public:

	// ����

	vector<PointNode>candidatePointNode;


	ARGS();
	~ARGS();

	// ѡ����������Ƭ
	Surface SelectSurface();


	// ��ȡ��ѡ�㼯
	vector<PointNode> GetCandidate(Surface seedSurface);

	// ��ѡ����Ӵ���
	vector<PointNode> AddCost(Surface seedSurface);

	// ��ѵ�ɸѡ
	vector<PointNode> GetBestPointNode(vector<PointNode> candidatePointNode);
};

