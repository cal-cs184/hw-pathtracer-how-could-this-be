#include "sphere.h"

#include <cmath>

#include "pathtracer/bsdf.h"
#include "util/sphere_drawing.h"

namespace CGL {
namespace SceneObjects {

bool Sphere::test(const Ray &r, double &t1, double &t2) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection test.
  // Return true if there are intersections and writing the
  // smaller of the two intersection times in t1 and the larger in t2.
  float a = dot(r.d, r.d);
  float b = 2 * dot(r.d, r.o - o);
  float c = dot((r.o - o), (r.o - o)) - this->r2;

  if (b * b - 4 * a * c >= 0) {
	  t1 = (-1.0 * b - sqrt(b * b - 4 * a * c)) / (2.0 * a);
	  t2 = (-1.0 * b + sqrt(b * b - 4 * a * c)) / (2.0 * a);
  }

  if (t1 >= r.min_t && t1 <= r.max_t) {
	  return true;
  }
  if (t2 >= r.min_t && t2 <= r.max_t) {
	  return true;
  }

  return false;

}

bool Sphere::has_intersection(const Ray &r) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note that you might want to use the the Sphere::test helper here.
  double t1 = -10, t2 = -10;

  return Sphere::test(r, t1, t2);
}

bool Sphere::intersect(const Ray &r, Intersection *i) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note again that you might want to use the the Sphere::test helper here.
  // When an intersection takes place, the Intersection data should be updated
  // correspondingly.
  double t1 = -10, t2 = -10;
  bool check = Sphere::test(r, t1, t2);

  if (check) {
	  r.max_t = t1;
	  i->bsdf = get_bsdf();
	  i->primitive = this;
	  if (t1 > 0) {
		  Vector3D n = r.o + t1 * r.d - this->o;
		  n /= n.norm();
		  i->n = n;
		  i->t = t1;
	  }
	  else {
		  Vector3D n = r.o + t2 * r.d - this->o;
		  n /= n.norm();
		  i->n = n;
		  i->t = t2;
	  }
  }

  return check;
}

void Sphere::draw(const Color &c, float alpha) const {
  Misc::draw_sphere_opengl(o, r, c);
}

void Sphere::drawOutline(const Color &c, float alpha) const {
  // Misc::draw_sphere_opengl(o, r, c);
}

} // namespace SceneObjects
} // namespace CGL
