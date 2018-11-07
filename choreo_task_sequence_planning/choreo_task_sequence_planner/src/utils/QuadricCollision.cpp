#include <cmath>

#include <Mathematics/GteTriangle.h>
#include <Mathematics/GteCylinder3.h>
#include <Mathematics/GteIntrSegment3Cone3.h>
#include <Mathematics/GteIntrSegment3Cylinder3.h>
#include <Mathematics/GteIntrSegment3Triangle3.h>
#include <Mathematics/GteDistSegmentSegment.h>

#include "choreo_task_sequence_planner/utils/GCommon.h"

#include "choreo_task_sequence_planner/utils/QuadricCollision.h"

static const double MAX_COLLISION_CHECK_DIST = 200; // mm

namespace{

double WF_edge_euclid_dist(WF_edge* eu, WF_edge* ev)
{
  auto u = eu->CenterPos();
  auto v = ev->CenterPos();

  double dx = u.x() - v.x();
  double dy = u.y() - v.y();
  double dz = u.z() - v.z();
  return sqrt(dx*dx + dy*dy + dz*dz);
}

gte::Segment<3, float> Seg(point target_start, point target_end)
{
  gte::Segment<3, float> segment;
  segment.p[0][0] = target_start.x();
  segment.p[0][1] = target_start.y();
  segment.p[0][2] = target_start.z();
  segment.p[1][0] = target_end.x();
  segment.p[1][1] = target_end.y();
  segment.p[1][2] = target_end.z();
  return segment;
}

gte::Segment<3, float> Seg(GeoV3 target_start, GeoV3 target_end)
{
  gte::Segment<3, float> segment;
  segment.p[0][0] = target_start.getX();
  segment.p[0][1] = target_start.getY();
  segment.p[0][2] = target_start.getZ();
  segment.p[1][0] = target_end.getX();
  segment.p[1][1] = target_end.getY();
  segment.p[1][2] = target_end.getZ();
  return segment;
}

gte::Triangle<3, float> Tri(GeoV3 a, GeoV3 b, GeoV3 c)
{
  gte::Triangle<3, float> triangle;
  triangle.v[0][0] = a.getX();
  triangle.v[0][1] = a.getY();
  triangle.v[0][2] = a.getZ();
  triangle.v[1][0] = b.getX();
  triangle.v[1][1] = b.getY();
  triangle.v[1][2] = b.getZ();
  triangle.v[2][0] = c.getX();
  triangle.v[2][1] = c.getY();
  triangle.v[2][2] = c.getZ();
  return triangle;
}

bool Parallel(GeoV3 a, GeoV3 b)
{
  if (abs(angle(a, b)) < GEO_EPS || abs(angle(a, b) - F_PI) < GEO_EPS)
  {
    return true;
  }

  return false;
}

// convert to eef direction 3d vector
GeoV3 Orientation(double theta, double phi)
{
  return GeoV3(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta));
}

Eigen::Vector3d ConvertAngleToEigenDirection(double theta, double phi)
{
  return Eigen::Vector3d(sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta));
}

} // anon util namespace

QuadricCollision::QuadricCollision()
{
  assert(THETA_DISC*PHI_DISC == DIR_SPHERE_DIVISION && "disc number is not compatible.");
  init_cmap_ = getInitCollisionMap();
}

int QuadricCollision::angleToCMapId(const double phi, const double theta)
{
  // we use theta-major(row) vector
  int i = std::round(phi/double(PHI_DISC));
  int j = std::round(theta/double(THETA_DISC));
  int id = i*THETA_DISC + j;

  assert(0 <= id && id < DIR_SPHERE_DIVISION);
  return id;
}

void QuadricCollision::cmapIDToAngle(const int id, double& phi, double& theta)
{
  // theta \in (0, pi), phi \in (0, 2pi)
  assert(0 <= id && id < DIR_SPHERE_DIVISION);

  int j = id % THETA_DISC;
  int i = (id - j) / THETA_DISC;

  assert(THETA_DISC > 1 && PHI_DISC > 1);

  phi = i * (2*F_PI/(double)PHI_DISC);
  theta =  j * (F_PI/(double)THETA_DISC);
}

EEDirArray QuadricCollision::getInitCollisionMap()
{
  EEDirArray cmap;
  cmap.fill(1);
  assert(cmap.size() == DIR_SPHERE_DIVISION);

  // construct the default
  // initial domain pruning, = 1 means free
  double phi, theta;
  for(int i = 0; i < cmap.size(); i++)
  {
    cmapIDToAngle(i, phi, theta);
    if(std::abs(theta) > F_PI - 75.0 / 180.0 * F_PI)
    {
      cmap[i] = 0;
    }
  }

  return cmap;
}

bool QuadricCollision::DetectCollision(const WF_edge *target_e, const WF_edge *exist_e, EEDirMap& result_map)
{
  result_map = init_cmap_;
  target_e_ = target_e;

  /* collision with edge */
//  return(DetectEdge(exist_e, result_map));
}

bool QuadricCollision::DetectCollision(WF_edge* target_e, WF_edge* exist_e,
                                       const EEDirArray& target_cmap,
                                       std::vector<lld>& result_cmap)
{
//  if(WF_edge_euclid_dist(target_e_, exist_e) > MAX_COLLISION_CHECK_DIST)
//  {
//    // if this existing edge is too far away from target_e_, we assume that
//    // there's no constraint arc between the target_e and exist_e
//
//    // return false if too far away
//    return false;
//  }

  // https://en.wikipedia.org/wiki/Spherical_coordinate_system
  // ISO naming convention (commonly used in physics)
  // azimuthal angle (rad)
  double phi;
  // polar angle theta (rad)
  double theta;

  result_cmap.clear();
  std::copy(target_cmap.begin(), target_cmap.end(), std::back_inserter(result_cmap));

  for(int i=0; i<target_cmap.size(); i++)
  {
    // check the dir that's still free
    if(1 == target_cmap[i])
    {
      cmapIDToAngle(i, phi, theta);

      if (DetectBulk(target_e, exist_e, phi, theta))
      {
        result_cmap[i] = 0;
      }
    }
  }

  for (int i = 0; i < 3; i++)
  {
    result_map[i] |= (*colli_map_[mi])[i];
  }

  return true;
}

bool QuadricCollision::DetectBulk(const WF_edge* target_e, const WF_edge* exist_e,
                                  const double phi, const double theta)
{
  assert(0<= theta && theta <= F_PI);
  assert(0<= phi && phi <= 2*F_PI);

  GeoV3 target_start = target_e->pvert_->Position();
  GeoV3 target_end = target_e->ppair_->pvert_->Position();

  GeoV3 exist_start = exist_e->pvert_->Position();
  GeoV3 exist_end = exist_e->ppair_->pvert_->Position();
  GeoV3 normal = Orientation(theta, phi);

  //0
  if (Parallel(normal, target_start - target_end))
  {
    if (ParallelCase(target_start, target_end, exist_start, exist_end, normal))
    {
      return true;
    }
    return false;
  }

  //1
  if ((target_start - exist_end).norm() < GEO_EPS)
  {
    if (SpecialCase(target_start,target_end,exist_start,normal ))
    {
      return true;
    }
    return false;
  }

  if ((target_start - exist_start).norm() < GEO_EPS)
  {
    if (SpecialCase(target_start, target_end, exist_end, normal))
    {
      return true;
    }
    return false;
  }

  if ((target_end - exist_end).norm() < GEO_EPS)
  {
    if (SpecialCase(target_end, target_start, exist_start, normal))
    {
      return true;
    }
    return false;
  }

  if ((target_end - exist_start).norm() < GEO_EPS)
  {
    if (SpecialCase(target_end, target_start, exist_end, normal))
    {
      return true;
    }
    return false;
  }

  //2
  if (Case(target_start, target_end, exist_start, exist_end, normal))
  {
    return true;
  }

  return false;
}

bool QuadricCollision::DetectAngle(GeoV3 connect, GeoV3 end, GeoV3 target_end, GeoV3 normal)
{
  if (angle(normal, target_end - connect) < extruder_.Angle())
    return true;
  return false;
}

bool QuadricCollision::Case(GeoV3 target_start, GeoV3 target_end,
                            GeoV3 exist_start, GeoV3 exist_end, GeoV3 normal)
{
  //Cone
  if (DetectCone(target_start, normal, exist_start, exist_end))
    return true;

  if (DetectCone(target_end, normal, exist_start, exist_end))
    return true;

  //Cylinder
  if (DetectCylinder(target_start, normal, exist_start, exist_end))
    return true;

  if (DetectCylinder(target_end, normal, exist_start, exist_end))
    return true;

  //Top
  if (DetectTopCylinder(target_start, normal, exist_start, exist_end))
    return true;
  if (DetectTopCylinder(target_end, normal, exist_start, exist_end))
    return true;

  //Face
  GenerateVolume(target_start, target_end, exist_start, exist_end, normal);
  for (int i = 0; i < bulk_.size(); i++)
  {
    if (DetectTriangle(bulk_[i], exist_start, exist_end))
      return true;
  }

  return false;
}

bool QuadricCollision::SpecialCase(GeoV3 connect, GeoV3 target_s, GeoV3 order_s, GeoV3 normal)
{
  if (angle(normal, order_s - connect) < extruder_.Angle())
  {
    return true;
  }

  if (DetectCone(target_s, normal, connect, order_s))
    return true;

  if (DetectCylinder(target_s, normal, connect, order_s))
    return true;

  //Top
  if (DetectTopCylinder(connect, normal, connect, order_s))
    return true;
  if (DetectTopCylinder(target_s, normal, connect, order_s))
    return true;

  //Face
  GenerateVolume(connect, target_s, order_s, normal);
  for (int i = 0; i < bulk_.size(); i++)
  {
    if (DetectTriangle(bulk_[i], connect, order_s))
      return true;
  }

  return false;
}

bool QuadricCollision::ParallelCase(GeoV3 target_start, GeoV3 target_end,
                                    GeoV3 exist_start, GeoV3 exist_end, GeoV3 normal)
{

  //Exception situation
  if ((target_start - exist_end).norm() < GEO_EPS)
  {
    if (DetectAngle(target_start, target_end, exist_start, normal))
    {
      return true;
    }
  }
  if ((target_start - exist_start).norm() < GEO_EPS)
  {
    if (DetectAngle(target_start, target_end, exist_end, normal))
    {

      return true;
    }
  }
  if ((target_end - exist_end).norm() < GEO_EPS)
  {
    if (DetectAngle(target_end, target_start, exist_start, normal))
    {

      return true;
    }
  }
  if ((target_end - exist_start).norm() < GEO_EPS)
  {
    if (DetectAngle(target_end, target_start, exist_end, normal))
    {

      return true;
    }
  }

  //Normal situation
  //Cone
  if (DetectCone(target_start, normal, exist_start, exist_end))
    return true;

  if (DetectCone(target_end, normal, exist_start, exist_end))
    return true;

  //Cylinder
  if (DetectCylinder(target_start, normal, exist_start, exist_end))
    return true;

  if (DetectCylinder(target_end, normal, exist_start, exist_end))
    return true;

  //Top
  if (DetectTopCylinder(target_start, normal, exist_start, exist_end))
    return true;
  if (DetectTopCylinder(target_end, normal, exist_start, exist_end))
    return true;

  return false;
}


bool QuadricCollision::DetectCone(GeoV3 start, GeoV3 normal, GeoV3 target_start, GeoV3 target_end)
{
  gte::Cone3<float> start_cone;
  start_cone.angle = extruder_.Angle();
  start_cone.height = extruder_.Height();

  gte::Ray3<float> start_ray;

  std::array<float, 3>s;
  s[0] = start.getX(); s[1] = start.getY(); s[2] = start.getZ();
  start_ray.origin = s;
  s[0] = normal.getX(); s[1] = normal.getY(); s[2] = normal.getZ();
  start_ray.direction = s;
  start_cone.ray = start_ray;
  gte::Segment<3, float> segment;
  segment = Seg(target_start, target_end);
  gte::FIQuery<float, gte::Segment<3, float>, gte::Cone3<float>> intersection;
  auto result = intersection(segment, start_cone);

  return result.intersect;
}

bool QuadricCollision::DetectCylinder(GeoV3 start, GeoV3 normal, GeoV3 target_start, GeoV3 target_end)
{
  gte::Cylinder3<float> cylinder;
  cylinder.axis;
  gte::Line3<float> cylinder_line;

  std::array<float, 3>s;

  GeoV3 cylin_center;
  cylin_center = start + normal * (extruder_.Height() + extruder_.CyclinderLenth()/2);

  s[0] = cylin_center.getX(); s[1] = cylin_center.getY(); s[2] = cylin_center.getZ();
  cylinder_line.origin = s;
  s[0] = normal.getX(); s[1] = normal.getY(); s[2] = normal.getZ();
  cylinder_line.direction = s;

  cylinder.axis = cylinder_line;

  cylinder.height = extruder_.CyclinderLenth();
  cylinder.radius = extruder_.Radii();

  gte::Segment<3, float> segment;
  segment = Seg(target_start, target_end);
  gte::FIQuery<float, gte::Segment<3, float>, gte::Cylinder3<float>> fiq;
  auto fiq_result = fiq(segment, cylinder);

  return fiq_result.intersect;
}

bool QuadricCollision::DetectTriangle(Triangle triangle, GeoV3 target_start, GeoV3 target_end)
{
  gte::Triangle<3, float> triangle_ = Tri(triangle.v0(), triangle.v1(), triangle.v2());

  gte::FIQuery<float, gte::Segment<3, float>, gte::Triangle3<float>> fiq;
  gte::Segment<3, float> segment = Seg(target_start, target_end);

  auto fiq_result = fiq(segment, triangle_);
  return fiq_result.intersect;
}

bool QuadricCollision::DetectTopCylinder(GeoV3 start, GeoV3 normal, GeoV3 target_start, GeoV3 target_end)
{
  gte::Cylinder3<float> cylinder;
  cylinder.axis;
  gte::Line3<float> cylinder_line;
  std::array<float, 3>s;
  GeoV3 cylin_center;
  cylin_center = start + normal*extruder_.TopCenter();
  s[0] = cylin_center.getX(); s[1] = cylin_center.getY(); s[2] = cylin_center.getZ();
  cylinder_line.origin = s;
  s[0] = normal.getX(); s[1] = normal.getY(); s[2] = normal.getZ();
  cylinder_line.direction = s;
  cylinder.axis = cylinder_line;

  cylinder.height = extruder_.ToolLenth();
  cylinder.radius = extruder_.TopRadii();


  gte::Segment<3, float> segment;
  segment = Seg(target_start, target_end);
  gte::FIQuery<float, gte::Segment<3, float>, gte::Cylinder3<float>> fiq;
  auto fiq_result = fiq(segment, cylinder);

  return fiq_result.intersect;
}

void QuadricCollision::GenerateVolume(GeoV3 start, GeoV3  end,
                                      GeoV3 target_start, GeoV3  target_end, GeoV3 normal)
{
  //face: front (up, down) back(up,down)
  GeoV3 t = end - start;
  double edge_length = t.norm();
  t.normalize();

  GeoV3 p = cross(t, normal);
  p.normalize();

  GeoV3 start_cone_center = normal*extruder_.Height() + start;
  GeoV3 end_cone_center = normal*extruder_.Height() + end;

  //face front
  GeoV3 start_front_cone = start_cone_center + p*extruder_.Radii();
  GeoV3 start_front_cylinder = start_front_cone + normal*extruder_.CyclinderLenth();
  GeoV3 end_front_cone = end_cone_center + p*extruder_.Radii();
  GeoV3 end_front_cylinder = end_front_cone + normal*extruder_.CyclinderLenth();

  //face back
  GeoV3 start_back_cone = start_cone_center - p*extruder_.Radii();
  GeoV3 start_back_cylinder = start_back_cone + normal*extruder_.CyclinderLenth();
  GeoV3 end_back_cone = end_cone_center - p*extruder_.Radii();
  GeoV3 end_back_cylinder = end_back_cone + normal*extruder_.CyclinderLenth();

  bulk_.clear();

  //Circle
  GeoV3 q = cross(normal, p);
  q.normalize();
  vector<GeoV3> start_circle_point,end_circle_point;
  for (int i = 0; i < 16; i++)
  {
    double part = 2 * F_PI / 16;
    double theta = i*part;
    start_circle_point.push_back(start_cone_center + p*cos(theta)*extruder_.Radii() + q*sin(theta)*extruder_.Radii());
    end_circle_point.push_back(start_circle_point[i] + end - start);
  }

  for (int i = 0; i < 15; i++)
  {
    bulk_.push_back(Triangle(start_circle_point[i], start_circle_point[i + 1], end_circle_point[i]));
    bulk_.push_back(Triangle(end_circle_point[i], end_circle_point[i + 1], start_circle_point[i + 1]));
  }
  bulk_.push_back(Triangle(start_circle_point[15], start_circle_point[0], end_circle_point[15]));
  bulk_.push_back(Triangle(end_circle_point[15], end_circle_point[0], start_circle_point[0]));

  for (int i = 0; i < 15; i++)
  {
    bulk_.push_back(Triangle(start_circle_point[i], start_circle_point[i + 1], end_circle_point[i]));
    bulk_.push_back(Triangle(end_circle_point[i], end_circle_point[i + 1], start_circle_point[i + 1]));
  }
  bulk_.push_back(Triangle(start_circle_point[15], start_circle_point[0], end_circle_point[15]));
  bulk_.push_back(Triangle(end_circle_point[15], end_circle_point[0], start_circle_point[0]));

  //Top face

  GeoV3 start_top_front_up = start + normal*(extruder_.TopLenth() / 2 + extruder_.TopCenter())+p*extruder_.TopRadii();
  GeoV3 start_top_front_down = start + normal*( extruder_.TopCenter() -extruder_.TopLenth() / 2 )+ p*extruder_.TopRadii();

  GeoV3 start_top_back_up =  start + normal*(extruder_.TopLenth() / 2 + extruder_.TopCenter()) - p*extruder_.TopRadii();
  GeoV3 start_top_back_down = start + normal*(extruder_.TopCenter() - extruder_.TopLenth() / 2) - p*extruder_.TopRadii();

  GeoV3 end_top_front_up = end + normal*(extruder_.TopLenth() / 2 + extruder_.TopCenter()) + p*extruder_.TopRadii();
  GeoV3 end_top_front_down = end + normal*(extruder_.TopCenter() - extruder_.TopLenth() / 2) + p*extruder_.TopRadii();

  GeoV3 end_top_back_up = end + normal*(extruder_.TopLenth() / 2 + extruder_.TopCenter()) - p*extruder_.TopRadii();
  GeoV3 end_top_back_down = end + normal*(extruder_.TopCenter() - extruder_.TopLenth() / 2) - p*extruder_.TopRadii();

  bulk_.push_back(Triangle(start_top_front_up, start_top_front_down, end_top_front_down));
  bulk_.push_back(Triangle(start_top_front_up, end_top_front_up, end_top_front_down));
  bulk_.push_back(Triangle(start_top_back_up, start_top_back_down, end_top_back_down));
  bulk_.push_back(Triangle(start_top_back_up, end_top_back_up, end_top_back_down));

  //front
  bulk_.push_back(Triangle(start, end, start_front_cone));
  bulk_.push_back(Triangle(end, end_front_cone, start_front_cone));
  bulk_.push_back(Triangle(start_front_cone, end_front_cone, start_front_cylinder));
  bulk_.push_back(Triangle(end_front_cone, end_front_cylinder, start_front_cylinder));

  //back
  bulk_.push_back(Triangle(start, end_back_cone, end));
  bulk_.push_back(Triangle(start, start_back_cone, end_back_cone));
  bulk_.push_back(Triangle(start_back_cone, end_back_cylinder, end_back_cone));
  bulk_.push_back(Triangle(start_back_cone, start_back_cylinder, end_back_cylinder));
}

void QuadricCollision::GenerateVolume(GeoV3 connect, GeoV3 target_s, GeoV3 order_s, GeoV3 normal)
{
  GeoV3 t = target_s - connect;
  double edge_length = t.norm();
  t.normalize();

  GeoV3 p = cross(t, normal);
  p.normalize();

  double eps = 1;

  GeoV3 start_cone_center = normal*extruder_.Height() + connect;

  //face front
  GeoV3 start_front_cone = start_cone_center + p*extruder_.Radii() + t*eps;
  GeoV3 start_front_cylinder = start_front_cone + normal*extruder_.CyclinderLenth() + t*eps;

  //face back
  GeoV3 start_back_cone = start_cone_center - p*extruder_.Radii() + t*eps;
  GeoV3 start_back_cylinder = start_back_cone + normal*extruder_.CyclinderLenth() + t*eps;
  GeoV3 start = connect + t*eps;

  //front
  bulk_.clear();
  bulk_.push_back(Triangle(start, start_back_cone, start_front_cone));
  bulk_.push_back(Triangle(start_front_cylinder, start_back_cone, start_front_cone));
  bulk_.push_back(Triangle(start_front_cylinder, start_back_cylinder, start_back_cone));

  //Circle
  start = connect;
  GeoV3 end = target_s;

  GeoV3 q = cross(normal, p);
  q.normalize();
  vector<GeoV3> start_circle_point, end_circle_point;
  for (int i = 0; i < 16; i++)
  {
    double part = 2 * F_PI / 16;
    double theta = i*part;
    start_circle_point.push_back(start_cone_center + p*cos(theta)*extruder_.Radii() + q*sin(theta)*extruder_.Radii());
    end_circle_point.push_back(start_circle_point[i] + end - start);
  }

  for (int i = 0; i < 15; i++)
  {
    bulk_.push_back(Triangle(start_circle_point[i], start_circle_point[i + 1], end_circle_point[i]));
    bulk_.push_back(Triangle(end_circle_point[i], end_circle_point[i + 1], start_circle_point[i + 1]));
  }
  bulk_.push_back(Triangle(start_circle_point[15], start_circle_point[0], end_circle_point[15]));
  bulk_.push_back(Triangle(end_circle_point[15], end_circle_point[0], start_circle_point[0]));

  //Top face
  GeoV3 start_top_front_up = start + normal*(extruder_.TopLenth() / 2 + extruder_.TopCenter()) + p*extruder_.TopRadii();
  GeoV3 start_top_front_down = start + normal*(extruder_.TopCenter() - extruder_.TopLenth() / 2) + p*extruder_.TopRadii();

  GeoV3 start_top_back_up = start + normal*(extruder_.TopLenth() / 2 + extruder_.TopCenter()) - p*extruder_.TopRadii();
  GeoV3 start_top_back_down = start + normal*(extruder_.TopCenter() - extruder_.TopLenth() / 2) - p*extruder_.TopRadii();

  GeoV3 end_top_front_up = end + normal*(extruder_.TopLenth() / 2 + extruder_.TopCenter()) + p*extruder_.TopRadii();
  GeoV3 end_top_front_down = end + normal*(extruder_.TopCenter() - extruder_.TopLenth() / 2) + p*extruder_.TopRadii();

  GeoV3 end_top_back_up = end + normal*(extruder_.TopLenth() / 2 + extruder_.TopCenter()) - p*extruder_.TopRadii();
  GeoV3 end_top_back_down = end + normal*(extruder_.TopCenter() - extruder_.TopLenth() / 2) - p*extruder_.TopRadii();

  bulk_.push_back(Triangle(start_top_front_up, start_top_front_down, end_top_front_down));
  bulk_.push_back(Triangle(start_top_front_up, end_top_front_up, end_top_front_down));
  bulk_.push_back(Triangle(start_top_back_up, start_top_back_down, end_top_back_down));
  bulk_.push_back(Triangle(start_top_back_up, end_top_back_up, end_top_back_down));
}

void QuadricCollision::ModifyAngle(const EEDirArray& cmap, EEDirArray& impacted_cmap)
{
  assert(cmap.size() == impacted_cmap.size());

  for (int i = 0; i < cmap.size(); i++)
  {
    // = 1 is free, = 0 blocked
    impacted_cmap[i] *= cmap[i];
  }
}

std::vector<Eigen::Vector3d> QuadricCollision::ConvertCollisionMapToEigenDirections(const EEDirArray& cmap)
{
  std::vector<Eigen::Vector3d> feasible_eef_directions;

  double phi, theta;

  for(int i=0; i < cmap.size(); i++)
  {
    if(cmap[i] == 1)
    {
      cmapIDToAngle(i, phi, theta);
      feasible_eef_directions.push_back(ConvertAngleToEigenDirection(theta, phi));
    }
  }

  return feasible_eef_directions;
}

