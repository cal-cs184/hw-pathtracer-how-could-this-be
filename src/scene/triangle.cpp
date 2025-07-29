#include "triangle.h"

#include "CGL/CGL.h"
#include "GL/glew.h"

namespace CGL {
namespace SceneObjects {

Triangle::Triangle(const Mesh *mesh, size_t v1, size_t v2, size_t v3) {
  p1 = mesh->positions[v1];
  p2 = mesh->positions[v2];
  p3 = mesh->positions[v3];
  n1 = mesh->normals[v1];
  n2 = mesh->normals[v2];
  n3 = mesh->normals[v3];
  bbox = BBox(p1);
  bbox.expand(p2);
  bbox.expand(p3);

  bsdf = mesh->get_bsdf();
}

BBox Triangle::get_bbox() const { return bbox; }

bool Triangle::has_intersection(const Ray &r) const {
  // Part 1, Task 3: implement ray-triangle intersection
  // The difference between this function and the next function is that the next
  // function records the "intersection" while this function only tests whether
  // there is a intersection.
  
  // Moller-Trumbore Algorithm
	Vector3D E1 = p2 - p1;
	Vector3D E2 = p3 - p1;
	Vector3D S = r.o - p1;
	Vector3D S1 = cross(r.d, E2);
	Vector3D S2 = cross(S, E1);

	Vector3D result = 1.0 / dot(S1, E1) * Vector3D(dot(S2, E2), dot(S1, S), dot(S2, r.d));

	float t = result.x;
	float b1 = result.y;
	float b2 = result.z;
	float b0 = 1.0 - b1 - b2;

	bool check = t >= r.min_t && t <= r.max_t && b0 >= 0 && b0 <= 1 && b1 >= 0 && b1 <= 1 && b2 >= 0 && b2 <= 1;

	return check;

}

bool Triangle::intersect(const Ray &r, Intersection *isect) const {
  // Part 1, Task 3:
  // implement ray-triangle intersection. When an intersection takes
  // place, the Intersection data should be updated accordingly

  // Moller-Trumbore Algorithm
  Vector3D E1 = p2 - p1;
  Vector3D E2 = p3 - p1;
  Vector3D S = r.o - p1;
  Vector3D S1 = cross(r.d, E2);
  Vector3D S2 = cross(S, E1);

  double divisor = dot(S1, E1);
  if (fabs(divisor) < EPS_F) return false; // or EPS_F
  Vector3D result = (1.0 / divisor) * Vector3D(dot(S2, E2), dot(S1, S), dot(S2, r.d));

  float t = result.x;
  float b1 = result.y;
  float b2 = result.z;
  float b0 = 1.0 - b1 - b2;

  bool check = t >= r.min_t && t <= isect->t && b0 >= 0 && b0 <= 1 && b1 >= 0 && b1 <= 1 && b2 >= 0 && b2 <= 1;

  if (check) {
	  //r.max_t = result.x; const
	  isect->t = result.x;
	  float a1 = 1.0 - result.y - result.z;
	  float a2 = result.y;
	  float a3 = result.z;
	  isect->n = a1 * n1 + a2 * n2 + a3 * n3;
	  isect->n /= isect->n.norm();
	  isect->primitive = this;
	  isect->bsdf = get_bsdf();
  }

  return check;

}

void Triangle::draw(const Color &c, float alpha) const {
  glColor4f(c.r, c.g, c.b, alpha);
  glBegin(GL_TRIANGLES);
  glVertex3d(p1.x, p1.y, p1.z);
  glVertex3d(p2.x, p2.y, p2.z);
  glVertex3d(p3.x, p3.y, p3.z);
  glEnd();
}

void Triangle::drawOutline(const Color &c, float alpha) const {
  glColor4f(c.r, c.g, c.b, alpha);
  glBegin(GL_LINE_LOOP);
  glVertex3d(p1.x, p1.y, p1.z);
  glVertex3d(p2.x, p2.y, p2.z);
  glVertex3d(p3.x, p3.y, p3.z);
  glEnd();
}

} // namespace SceneObjects
} // namespace CGL
