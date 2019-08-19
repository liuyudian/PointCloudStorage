#include "Surface.h"
Surface::Surface()
{
}

Surface::Surface(CEdge edge1, CEdge edge2, CEdge edge3)
{
	this->edge1 = edge1;
	this->edge2 = edge2;
	this->edge3 = edge3;
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
	return false;
}


void Surface::GetAngle()
{
	double currentAngle = 0;
	double len_ab = sqrt(pow((p0.x - p1.x), 2.0) + pow((p0.y - p1.y), 2.0) + pow((p0.z - p1.z), 2.0));

	double len_as = sqrt(pow((p0.x - p2.x), 2.0) + pow((p0.y - p2.y), 2.0) + pow((p0.z - p2.z), 2.0));
	double len_bs = sqrt(pow((p2.x - p1.x), 2.0) + pow((p2.y - p1.y), 2.0) + pow((p2.z - p1.z), 2.0));

	// 周长

	double A = acos((len_ab*len_ab + len_as * len_as - len_bs * len_bs) / (2 * len_ab*len_as));

	double B = acos((len_ab*len_ab + len_bs * len_bs - len_as * len_as) / (2 * len_ab*len_bs));

	double S = acos((len_as*len_as + len_bs * len_bs - len_ab * len_ab) / (2 * len_as*len_bs));


	if (A > B)
	{
		if (A > S)
		{
			currentAngle = A;
		}
		else
		{
			currentAngle = S;
		}
	}
	else
	{
		if (B > S)
		{
			currentAngle = B;
		}
		else
		{
			currentAngle = S;
		}
	}

	if (currentAngle > 90)
	{
		this->angle = 1;
	}
}
void Surface::Normal()
{
		// 存放法矢
		vector<double>list;
		double v1x = p1.x - p0.x;
		double v1y = p1.y - p0.y;
		double v1z = p1.z - p0.z;
		double v2x = p2.x - p1.x;
		double v2y = p2.y - p1.y;
		double v2z = p2.z - p1.z;
		double x = v1y * v2z - v1z * v2y;
		double y = v1z * v2x - v1x * v2z;
		double z = v1x * v2y - v1y * v2x;
		double len = sqrt(x*x + y * y + z * z);
		if (len != 0)
		{
			list.push_back(x / len);
			list.push_back(y / len);
			list.push_back(z / len);
		}

}
