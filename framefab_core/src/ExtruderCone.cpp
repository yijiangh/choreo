#include"ExtruderCone.h"

ExtruderCone::ExtruderCone()
{
	/*
	* Extruder Data:
	* Update: Dec/08/2015
	* angle(radians) : angle between generatrix and central axis
	* height(cm)	 : length of the central axis
	* wave_angle	 : maximal waving angle for rotating orientation
	* divide		 : used in Collision Detection Algo. Ver1.0
					   the number of triangle face to approx. the sweeping
	*				   cone solid
	*/
	normal_		= Vec3f(0, 0, 1);
	angle_ = ((double)30 / (double)180) * F_PI;
	height_			= 20;
	wave_angle_ = F_PI / 18;
	divide_			= 16;
	tool_lenth_	= 15;// 120mm connect robot arm
	radii_			= tan(angle_)*height_;
	cyclinder_height_= 100;//50mm for connection tools on the arm


	//top_cylin
	top_cylin_center_lenth_ = 89; //from start point
	top_cylin_lenth_			= 68;
	top_cylin_radii_				= 36;

	GeneCone();
}

ExtruderCone::ExtruderCone(double height, point  base_point, Vec3f normal, double angle)
{
	height_ = height;
	base_point_ = base_point;
	normal_ = normal;
	angle_ = angle;
}

ExtruderCone::~ExtruderCone()
{
	
}

void ExtruderCone::Test()
{
	cout << angle_ << " " << normal_.x() << normal_.z()<<endl;
}

void ExtruderCone::GeneCone()
{
	vector<Triangle> temp;
	vector<point>    ploy;
	double radii = tan(angle_)*height_;
	for (int i = 0; i < divide_; i++)
	{
		point v0 = base_point_;
		point v1 = point(radii*cos(2 * F_PI / divide_*(i + 1)), radii*sin(2 * F_PI / divide_*(i + 1)), height_);
		point v2 = point(radii*cos(2 * F_PI / divide_*(i)), radii*sin(2 * F_PI / divide_*(i)), height_);

		ploy.push_back(v2);
		temp.push_back(Triangle(base_point_, v1, v2));
	}
	side_	  = temp;
	side_end_ = side_;
	top_	  = ploy;
}

void ExtruderCone::Render(WireFrame* ptr_frame, double alpha)
{

	for (int i = 0; i < side_.size(); i++)
	{
		side_[i].Render(ptr_frame, alpha);
		side_end_[i].Render(ptr_frame, alpha);
	}

	//render top

	glBegin(GL_POLYGON);
	glColor4f(0.5, 0.5, 0.7, alpha);
	glNormal3fv(normal_);
		for (int i = 0; i < top_.size(); i++)
		{
			glVertex3fv(ptr_frame->Unify(top_[i]));
		}
	glEnd();

	glBegin(GL_QUADS);
	glColor4f(0.5, 0.5, 0.7, alpha);
	for (int i = 0; i < side_.size(); i++)
	{
		for (int j = 0; j < 3; j++)
		{
			glVertex3fv(ptr_frame->Unify(side_[i].vert_list_[j]));
			glVertex3fv(ptr_frame->Unify(side_end_[i].vert_list_[j]));
			glVertex3fv(ptr_frame->Unify(end_));
			glVertex3fv(ptr_frame->Unify(start_));
		}
	}
	glEnd();


}

void ExtruderCone::Rotation(double angle, point start, point end)
{
	
	if (start.z() >= end.z())
	{
		point temp = start;
		start = end;
		end = temp;
	}
	start_ = start;
	end_ = end;


	float a =start.x();
	float b = start.y();
	float c = start.z();
	Geometry::Vector3d p= Geometry::Vector3d( point(end - start));
	
	p.normalize();
    if (p.getZ() == 1)
		return;

	Geometry::Vector3d pz = Geometry::cross(p, Geometry::Vector3d(0, 0, 1));
	pz.normalize();
	Geometry::Vector3d z = Geometry::Vector3d(0, 0, 1);
	Geometry::Vector3d zpz = Geometry::cross(z, pz);
	zpz.normalize();
	Geometry::Vector3d vec = pz * cos(angle) + z * sin(angle);
	normal_ = Vec3f(vec.getX(), vec.getY(), vec.getZ());

	Geometry::Vector3d u = pz*sin(angle) -z*cos(angle);

	double radii = tan(angle_)*height_;

	for (int i = 0; i < divide_; i++)
	{
		
		Geometry::Vector3d v1 = vec*height_ + u*(radii*cos(2 * F_PI / divide_*(i + 1))) + zpz*(radii*sin(2 * F_PI / divide_*(i + 1)));
		point v1_(v1.getX(), v1.getY(), v1.getZ());
	
		Geometry::Vector3d v2 = vec*height_ + u*(radii*cos(2 * F_PI / divide_*(i))) + zpz*(radii*sin(2 * F_PI / divide_*(i)));
		point v2_(v2.getX(), v2.getY(), v2.getZ());
		top_[i] = v2_;
		side_[i] = Triangle(base_point_, v1_, v2_);
	}

	side_end_ = side_;
	// Move
	for (int i = 0; i < side_.size(); i++)
	{
		side_[i].Add(end);
		side_end_[i].Add(start);
	}
		

	for (int i = 0; i < top_.size(); i++)
		top_[i] += end;
}

void ExtruderCone::RotateTri(Triangle temp)
{
	for (int i = 0; i < temp.vert_list_.size(); i++)
	{
		temp.vert_list_[i] = Multi(temp.vert_list_[i]);
	}
	temp.Normal_();
}

point ExtruderCone::Multi(point s)
{

	point temp = s;
	double x = rotate_[0][0] * temp.x() + rotate_[0][1] * temp.y(); +rotate_[0][2] * temp.z() + rotate_[3][0];
	double y = rotate_[1][0] * temp.x() + rotate_[1][1] * temp.y(); +rotate_[1][2] * temp.z() + rotate_[3][1];
	double z = rotate_[2][0] * temp.x() + rotate_[2][1] * temp.y(); +rotate_[2][2] * temp.z() + rotate_[3][2];
	
	return point(x, y, z);
}


void ExtruderCone::Rotation(GeoV3 normal, point start, point end)
{
	normal_ = Vec3f(normal.getX(),normal.getY(),normal.getZ());



	if (start.z() >= end.z())
	{
		point temp = start;
		start = end;
		end = temp;
	}
	start_ = start;
	end_ = end;


	float a = start.x();
	float b = start.y();
	float c = start.z();
	Geometry::Vector3d p = Geometry::Vector3d(point(end - start));

	p.normalize();

	if (Geometry::dot(p, normal) > 0.001)
		return;


	if (p.getZ() == 1)
	{
		normal = GeoV3(0, 0, 1);
		Geometry::Vector3d pz = Geometry::cross(p, normal);
		pz.normalize();
		Geometry::Vector3d u = Geometry::Vector3d(0, 1, 0);
		Geometry::Vector3d v = Geometry::Vector3d(1, 0, 0);
		double radii = tan(angle_)*height_;

		for (int i = 0; i < divide_; i++)
		{
			Geometry::Vector3d v1 = Geometry::Vector3d(base_point_) + normal*height_
                                    + u*(radii*cos(2 * F_PI / divide_*(i + 1))) + v*(radii*sin(2 * F_PI / divide_*(i + 1)));
			point v1_(v1.getX(), v1.getY(), v1.getZ());

			Geometry::Vector3d v2 = Geometry::Vector3d(base_point_) + normal*height_
                                    + u*(radii*cos(2 * F_PI / divide_*(i))) + v*(radii*sin(2 * F_PI / divide_*(i)));
			point v2_(v2.getX(), v2.getY(), v2.getZ());
			top_[i] = v2_;

			side_[i] = Triangle(base_point_, v1_, v2_);
		}
		side_end_ = side_;
		// Move
		for (int i = 0; i < side_.size(); i++)
		{
			side_[i].Add(end);
			side_end_[i].Add(start);
		}
		for (int i = 0; i < top_.size(); i++)
			top_[i] += end;
	}
	else
	{
		Geometry::Vector3d pz = Geometry::cross(p, normal);
		pz.normalize();
		Geometry::Vector3d u = p;
		Geometry::Vector3d v = pz;
		double radii = tan(angle_)*height_;
		for (int i = 0; i < divide_; i++)
		{
			Geometry::Vector3d v1 = Geometry::Vector3d(base_point_) + normal * height_
                                    + u*(radii*cos(2 * F_PI / divide_*(i + 1))) + v*(radii*sin(2 * F_PI / divide_*(i + 1)));
			point v1_(v1.getX(), v1.getY(), v1.getZ());

            Geometry::Vector3d v2 = Geometry::Vector3d(base_point_) + normal*height_
                                    + u*(radii*cos(2 * F_PI / divide_*(i))) + v*(radii*sin(2 * F_PI / divide_*(i)));
			point v2_(v2.getX(), v2.getY(), v2.getZ());

            top_[i] = v2_;

			side_[i] = Triangle(base_point_, v1_, v2_);
		}
		side_end_ = side_;
		// Move
		for (int i = 0; i < side_.size(); i++)
		{
			side_[i].Add(end);
			side_end_[i].Add(start);
		}
		for (int i = 0; i < top_.size(); i++)
			top_[i] += end;
	}
}