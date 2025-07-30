#include "pathtracer.h"

#include "scene/light.h"
#include "scene/sphere.h"
#include "scene/triangle.h"


using namespace CGL::SceneObjects;

namespace CGL {

PathTracer::PathTracer() {
  gridSampler = new UniformGridSampler2D();
  hemisphereSampler = new UniformHemisphereSampler3D();

  tm_gamma = 2.2f;
  tm_level = 1.0f;
  tm_key = 0.18;
  tm_wht = 5.0f;
}

PathTracer::~PathTracer() {
  delete gridSampler;
  delete hemisphereSampler;
}

void PathTracer::set_frame_size(size_t width, size_t height) {
  sampleBuffer.resize(width, height);
  sampleCountBuffer.resize(width * height);
}

void PathTracer::clear() {
  bvh = NULL;
  scene = NULL;
  camera = NULL;
  sampleBuffer.clear();
  sampleCountBuffer.clear();
  sampleBuffer.resize(0, 0);
  sampleCountBuffer.resize(0, 0);
}

void PathTracer::write_to_framebuffer(ImageBuffer &framebuffer, size_t x0,
                                      size_t y0, size_t x1, size_t y1) {
  sampleBuffer.toColor(framebuffer, x0, y0, x1, y1);
}

Vector3D
PathTracer::estimate_direct_lighting_hemisphere(const Ray &r,
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // For this function, sample uniformly in a hemisphere.

  // Note: When comparing Cornel Box (CBxxx.dae) results to importance sampling, you may find the "glow" around the light source is gone.
  // This is totally fine: the area lights in importance sampling has directionality, however in hemisphere sampling we don't model this behaviour.

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D hit_p = r.o + r.d * isect.t;
  const Vector3D w_out = w2o * (-r.d);

  // This is the same number of total samples as
  // estimate_direct_lighting_importance (outside of delta lights). We keep the
  // same number of samples for clarity of comparison.
  int num_samples = scene->lights.size() * ns_area_light;
  Vector3D L_out;

  // TODO (Part 3): Write your sampling loop here
  // TODO BEFORE YOU BEGIN
  // UPDATE `est_radiance_global_illumination` to return direct lighting instead of normal shading 
  float pdf = (2.0 * PI);
  for (int i = 0; i < num_samples; i++) {
      // Sample direction in local space
      Vector3D wi = hemisphereSampler->get_sample();
	  wi /= wi.norm(); // Normalize the sampled direction

      // Cosine term
      double cos_theta = std::max(wi.z, 0.0);

      // Convert direction to world space and shoot a ray
      Vector3D wi_world = o2w * wi;
      Ray shadow_ray(hit_p, wi_world);
      shadow_ray.min_t = EPS_F;
      shadow_ray.max_t = INF_F;

      Intersection light_isect;
      if (bvh->intersect(shadow_ray, &light_isect)) {
          Vector3D emission = light_isect.bsdf->get_emission();
          if (!(emission == Vector3D())) {
              Vector3D f = isect.bsdf->f(w_out, wi);
              L_out += emission * f * cos_theta * pdf;
          }
      }
  }

  return L_out / num_samples;

}

Vector3D
PathTracer::estimate_direct_lighting_importance(const Ray& r,
    const Intersection& isect) {
    // Make a coordinate system for a hit point with N aligned with the Z direction.
    Matrix3x3 o2w;
    make_coord_space(o2w, isect.n);
    Matrix3x3 w2o = o2w.T();

    const Vector3D hit_p = r.o + r.d * isect.t;
    const Vector3D w_out = w2o * (-r.d);
    Vector3D L_out(0, 0, 0);

    for (size_t i = 0; i < scene->lights.size(); ++i) {
        SceneLight* light = scene->lights[i];
        int n_samples = light->is_delta_light() ? 1 : ns_area_light;

        for (int j = 0; j < n_samples; ++j) {
            Vector3D wi_world, L_sample;
            double distToLight, pdf;

            L_sample = light->sample_L(hit_p, &wi_world, &distToLight, &pdf);
            if (pdf <= 0.0) continue;

            // Convert wi_world to local space
            Vector3D wi = w2o * wi_world;

            // Cosine term
            double cos_theta = std::max(wi.z, 0.0);

            // Shadow ray
            Ray shadow_ray(hit_p, wi_world);
            shadow_ray.min_t = EPS_F;
            shadow_ray.max_t = distToLight - EPS_F;

            Intersection shadow_isect;
            if (!bvh->intersect(shadow_ray, &shadow_isect)) {
                // No occlusion
                Vector3D f = isect.bsdf->f(w_out, wi);
                L_out += L_sample * f * cos_theta / pdf;
            }
        }
        if (n_samples > 1)
			L_out /= n_samples;
    }

    return L_out;
}

Vector3D PathTracer::zero_bounce_radiance(const Ray &r,
                                          const Intersection &isect) {
  // Returns the light that results from no bounces of light

  return isect.bsdf->get_emission();


}

Vector3D PathTracer::one_bounce_radiance(const Ray& r,
    const Intersection& isect) {
    // Returns either the direct illumination by hemisphere or importance sampling
    // depending on `direct_hemisphere_sample`
    Vector3D direct;
    if (direct_hemisphere_sample) {
        direct = estimate_direct_lighting_hemisphere(r, isect);
    }
    else {
        direct = estimate_direct_lighting_importance(r, isect);
    }
    return direct;
}

Vector3D PathTracer::at_least_one_bounce_radiance(const Ray &r, const Intersection &isect) {
    Vector3D L_out;

    if (r.depth >= max_ray_depth - 1) {
        L_out = estimate_direct_lighting_importance(r, isect);
        return L_out;
    }

    //double rr_prob = 0.6;
    //if (r.depth > 0 && coin_flip(rr_prob)) return L_out;

    // Sample BSDF
    Matrix3x3 o2w; make_coord_space(o2w, isect.n);
    Matrix3x3 w2o = o2w.T();
    Vector3D w_out = w2o * (-r.d);

    Vector3D wi; double pdf;
    Vector3D f = isect.bsdf->sample_f(w_out, &wi, &pdf);
    if (pdf < EPS_F || f == Vector3D()) return L_out;

    Vector3D wi_world = o2w * wi; wi_world.normalize();
    Ray bounce_ray(isect.t * r.d + r.o, wi_world, INF_F, r.depth + 1);
    bounce_ray.min_t = EPS_F;

    Intersection bounce_isect;
    if (bvh->intersect(bounce_ray, &bounce_isect)) {
        double cos_theta = std::max(wi.z, 0.0);
        Vector3D indirect = at_least_one_bounce_radiance(bounce_ray, bounce_isect);
        //indirect /= rr_prob; // scale up surviving rays
        if (isAccumBounces)
            L_out = f * indirect * cos_theta / pdf + estimate_direct_lighting_importance(r, isect);
        else
			L_out = f * indirect * cos_theta / pdf; // only indirect lighting
    }
    return L_out;
}

Vector3D PathTracer::est_radiance_global_illumination(const Ray &r) {
  Intersection isect;
  Vector3D L_out;

  // You will extend this in assignment 3-2.
  // If no intersection occurs, we simply return black.
  // This changes if you implement hemispherical lighting for extra credit.

  // The following line of code returns a debug color depending
  // on whether ray intersection with triangles or spheres has
  // been implemented.
  //
  // REMOVE THIS LINE when you are ready to begin Part 3.
  if (!bvh->intersect(r, &isect))
    return envLight ? envLight->sample_dir(r) : L_out;

  if (isAccumBounces)
    L_out = PathTracer::at_least_one_bounce_radiance(r, isect) + zero_bounce_radiance(r, isect);
  else if (max_ray_depth > 0)
	L_out = PathTracer::at_least_one_bounce_radiance(r, isect);
  else 
	L_out = zero_bounce_radiance(r, isect);
  //L_out = (isect.t == INF_D) ? debug_shading(r.d) : normal_shading(isect.n);

  // TODO (Part 3): Return the direct illumination.

  // TODO (Part 4): Accumulate the "direct" and "indirect"
  // parts of global illumination into L_out rather than just direct

  return L_out;
}

void PathTracer::raytrace_pixel(size_t x, size_t y) {
    // Adaptive sampling parameters
    int max_samples = ns_aa; // maximum samples per pixel
    int samplesPerBatch = this->samplesPerBatch; // batch size (default 32)
    double maxTolerance = this->maxTolerance;    // tolerance (default 0.05)

    Vector3D t(0, 0, 0); // accumulated radiance
    double s1 = 0.0;     // sum of illuminance
    double s2 = 0.0;     // sum of illuminance squared
    int n = 0;           // number of samples taken

    for (; n < max_samples;) {
        // Take a batch of samples
        for (int k = 0; k < samplesPerBatch && n < max_samples; ++k, ++n) {
            Vector2D coord = gridSampler->get_sample();
            double nx = (coord.x + x) / (double)sampleBuffer.w;
            double ny = (coord.y + y) / (double)sampleBuffer.h;
            Ray r = camera->generate_ray(nx, ny);
            Vector3D radiance = est_radiance_global_illumination(r);
            t += radiance;

            double illum = radiance.illum();
            s1 += illum;
            s2 += illum * illum;
        }

        // Check convergence every batch, but not on the first batch
        if (n >= samplesPerBatch) {
            double mu = s1 / n;
            double sigma2 = (n > 1) ? (s2 - s1 * s1 / n) / (n - 1) : 0.0;
            double sigma = sqrt(std::max(0.0, sigma2));
            double I = 1.96 * sigma / sqrt((double)n);

            if (I <= maxTolerance * mu) {
                break; // pixel has converged
            }
        }
    }

    t /= n; // average radiance
    sampleBuffer.update_pixel(t, x, y);
    sampleCountBuffer[x + y * sampleBuffer.w] = n;
}

void PathTracer::autofocus(Vector2D loc) {
  Ray r = camera->generate_ray(loc.x / sampleBuffer.w, loc.y / sampleBuffer.h);
  Intersection isect;

  bvh->intersect(r, &isect);

  camera->focalDistance = isect.t;
}

} // namespace CGL
