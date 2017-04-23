#include "ResolveAngle.h"



ResolveAngle::ResolveAngle(vector< Geometry::Vector3d> list)
{
	list_ = list;
	a_ = Resolve();
	if (list_.size() > 0)
		b_ = Resolve();
	if (list_.size() > 0)
		c_ = Resolve();
	Dec();
}

ResolveAngle::ResolveAngle()
{

}


vector< Geometry::Vector3d> ResolveAngle::Resolve()
{
	vector< Geometry::Vector3d> m;
	Geometry::Vector3d temp;
	temp = *list_.begin();
	
	vector< Geometry::Vector3d>::iterator it;
	for (it = list_.begin(); it != list_.end();)
	{
		Geometry::Vector3d target = *it;
		double s= Geometry::angle(target, temp);
		if (Geometry::angle(target, temp) < extruder_.Angle())
		{
			m.push_back(target);
			temp = target;
			it=list_.erase(it);
		}
		else
		{
			break;
		}
	}

	if (list_.size() >0 )
	{
		for (it = list_.end()-1; it != list_.begin();)
		{
			Geometry::Vector3d target = *it;
			if (Geometry::angle(target, temp) < extruder_.Angle())
			{
				m.push_back(target);
				temp = target;
				list_.erase(it);
				it = list_.end() - 1;
			}
			else
			{
				break;
			}
		}
	}
	return m;
}

ResolveAngle::~ResolveAngle()
{
	a_.clear();
	b_.clear();
	c_.clear();

}

void ResolveAngle::Dec()
{
	vector<Geometry::Vector3d> max;
	max = a_;
	if (max.size() < b_.size())
		max = b_;
	
	if (max.size() < c_.size())
		max = c_;

	if (max.size() == 1)
	{
		dec = max[0];
		wave = 0;
		return;
	}


	if (max.size() ==72)
	{
		int id = 0;
		double z_max = -1;
		for (int i = 0; i < max.size(); i++)
		{
			if (z_max < max[i].getZ())
			{
				z_max = max[i].getZ();
				id = i;
			}
		}


		dec = max[id];
		wave = 2 * F_PI;
		return;

	}


	Geometry::Vector3d temp_max(0,0,0);

	for (int i = 0; i < max.size(); i++)
	{
		temp_max =temp_max+ max[i];
	}
	temp_max.normalize();
	dec = temp_max;
	wave = 0;
	for (int i = 0; i < max.size(); i++)
	{
		if (Geometry::angle(temp_max, max[i])>wave)
			wave = Geometry::angle(temp_max, max[i]);

	}

}