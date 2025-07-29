#include "bvh.h"

#include "CGL/CGL.h"
#include "triangle.h"

#include <iostream>
#include <stack>

using namespace std;

namespace CGL {
namespace SceneObjects {

BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
                   size_t max_leaf_size) {

  primitives = std::vector<Primitive *>(_primitives);
  root = construct_bvh(primitives.begin(), primitives.end(), max_leaf_size);
}

BVHAccel::~BVHAccel() {
  if (root)
    delete root;
  primitives.clear();
}

BBox BVHAccel::get_bbox() const { return root->bb; }

void BVHAccel::draw(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->draw(c, alpha);
    }
  } else {
    draw(node->l, c, alpha);
    draw(node->r, c, alpha);
  }
}

void BVHAccel::drawOutline(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->drawOutline(c, alpha);
    }
  } else {
    drawOutline(node->l, c, alpha);
    drawOutline(node->r, c, alpha);
  }
}

BVHNode *BVHAccel::construct_bvh(std::vector<Primitive *>::iterator start,
                                 std::vector<Primitive *>::iterator end,
                                 size_t max_leaf_size) {

  // TODO (Part 2.1):
  // Construct a BVH from the given vector of primitives and maximum leaf
  // size configuration. The starter code build a BVH aggregate with a
  // single leaf node (which is also the root) that encloses all the
  // primitives.

  // get longest axis
  // sort into two halves
  // call on left and right
    size_t n = end - start;
    BBox bbox;
    std::vector<Vector3D> centroids;

    for (auto it = start; it != end; ++it) {
        BBox bb = (*it)->get_bbox();
        bbox.expand(bb);
        centroids.push_back(bb.centroid());
    }

    if (n <= max_leaf_size) {
        BVHNode* leaf = new BVHNode(bbox);
        leaf->start = start;
        leaf->end = end;
        leaf->l = nullptr;
        leaf->r = nullptr;
        return leaf;
    }

    // Compute bounding box of centroids to choose axis
    BBox centroid_bbox;
    for (const auto& c : centroids)
        centroid_bbox.expand(c);

    Vector3D extent = centroid_bbox.extent;
    int axis = 0;
    if (extent.y > extent.x) axis = 1;
    if (extent.z > extent[axis]) axis = 2;

    // Sort by centroid along the chosen axis using lamdba function defining comparison
    std::sort(start, end, [axis](Primitive* a, Primitive* b) {
        return a->get_bbox().centroid()[axis] < b->get_bbox().centroid()[axis];
    });

    auto mid = start + n / 2;
    BVHNode* node = new BVHNode(bbox);
    node->l = construct_bvh(start, mid, max_leaf_size);
    node->r = construct_bvh(mid, end, max_leaf_size);
    node->start = start;
    node->end = end;
    return node;

}

bool BVHAccel::has_intersection(const Ray &ray, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.
  // Take note that this function has a short-circuit that the
  // Intersection version cannot, since it returns as soon as it finds
  // a hit, it doesn't actually have to find the closest hit.
    bool hit = false;

    if (!node) return false;

    if (node->l == nullptr && node->r == nullptr) { // Leaf node
        for (auto p = node->start; p != node->end; ++p) {
            if ((*p)->has_intersection(ray))
                return true;
        }
        return false;
    }

    // Internal node
    double t0_l = 0, t1_l = EPS_F;
    double t0_r = 0, t1_r = EPS_F;
    bool hit_l = node->l && node->l->bb.intersect(ray, t0_l, t1_l);
    bool hit_r = node->r && node->r->bb.intersect(ray, t0_r, t1_r);


    if (hit_l)
        hit = has_intersection(ray, node->l) || hit;
    if (hit_r)
        hit = has_intersection(ray, node->r) || hit;

    return hit;

}

bool BVHAccel::intersect(const Ray &ray, Intersection *i, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.
  //    bool hit = false;
  //for (auto p : primitives) {
  //  total_isects++;
  //  hit = p->intersect(ray, i) || hit;
  //}
  //return hit;

    bool hit = false;

    if (!node) return false;

    if (node->l == nullptr && node->r == nullptr) { // Leaf node
        for (auto p = node->start; p != node->end; ++p) {
            total_isects++;
            hit = (*p)->intersect(ray, i) || hit;
        }
        return hit;
    }

    // Internal node
    double t0_l = 0, t1_l = EPS_F;
    double t0_r = 0, t1_r = EPS_F;
    bool hit_l = node->l && node->l->bb.intersect(ray, t0_l, t1_l);
    bool hit_r = node->r && node->r->bb.intersect(ray, t0_r, t1_r);


    if (hit_l) 
        hit = intersect(ray, i, node->l) || hit;
    if (hit_r) 
        hit = intersect(ray, i, node->r) || hit;

    return hit;

}

} // namespace SceneObjects
} // namespace CGL
