#pragma once
#include <iostream>
using namespace std;
//����˲����ڵ���
template<class T>
struct OctreeNode 
{
	T data; //�ڵ�����
	T xMin, xMax; //�ڵ����꣬������������������
	T yMin, yMax;
	T zMin, zMax;
	OctreeNode <T> *top_left_front, *top_left_back; //�ýڵ�ĸ��ӽ��
	OctreeNode <T> *top_right_front, *top_right_back;
	OctreeNode <T> *bottom_left_front, *bottom_left_back;
	OctreeNode <T> *bottom_right_front, *bottom_right_back;
	OctreeNode //�ڵ���
};
