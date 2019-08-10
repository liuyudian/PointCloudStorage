#include "Surface.h"
Surface::Surface()
{
}


Surface::~Surface()
{
}

float Surface::GetMaxLen()
{
// 获取最大边的长度
	float L = 0.0f;
	if (edge1.len > edge2.len)
	{
		if (edge1.len > edge3.len)
		{
			L = edge1.len;
		}
		else
		{
			L = edge3.len;
		}
	}
	else
	{
		if (edge2.len > edge3.len)
		{
			L = edge2.len;
		}
		else
		{
			L = edge3.len;
		}
	}
	return L;
}

// 判断点是否在三角面片之中
bool Surface::isWithin(pcl::PointXYZ p)
{
	
	/*return  (bx - ax) * (y - ay) > (by - ay) * (x - ax) &&
		(cx - bx) * (y - by) > (cy - by) * (x - bx) &&
		(ax - cx) * (y - cy) > (ay - cy) * (x - cx) ? false : true;*/
}
