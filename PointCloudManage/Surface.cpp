#include "Surface.h"
Surface::Surface()
{
}


Surface::~Surface()
{
}

float Surface::GetMaxLen()
{
// ��ȡ���ߵĳ���
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

// �жϵ��Ƿ���������Ƭ֮��
bool Surface::isWithin(pcl::PointXYZ p)
{
	
	/*return  (bx - ax) * (y - ay) > (by - ay) * (x - ax) &&
		(cx - bx) * (y - by) > (cy - by) * (x - bx) &&
		(ax - cx) * (y - cy) > (ay - cy) * (x - cx) ? false : true;*/
}
