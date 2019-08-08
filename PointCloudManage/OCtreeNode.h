#pragma once
#include <iostream>
using namespace std;
//定义八叉树节点类
template<class T>
struct OctreeNode 
{
	T data; //节点数据
	T xMin, xMax; //节点坐标，即六面体个顶点的坐标
	T yMin, yMax;
	T zMin, zMax;
	OctreeNode <T> *top_left_front, *top_left_back; //该节点的个子结点
	OctreeNode <T> *top_right_front, *top_right_back;
	OctreeNode <T> *bottom_left_front, *bottom_left_back;
	OctreeNode <T> *bottom_right_front, *bottom_right_back;
	OctreeNode //节点类
};
