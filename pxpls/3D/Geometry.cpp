//
//  Geometry.cpp
//  pxpls
//
//  Created by Ninter6 on 2024/2/19.
//

#include "Geometry.hpp"

#include <cmath>
#include <limits>

#define EPSILON std::numeric_limits<float>::epsilon()
#define LESS(a, b) b - a > EPSILON
#define LESS_EQUAL(a, b) a - b <= EPSILON
#define EQUAL(a, b) std::abs(a - b) <= EPSILON

namespace pxpls {

Sphere::Sphere(const Point& center, float radius) : center(center), radius(radius) {}

Plane::Plane(const Point& center, const mathpls::vec3& normal)
: center(center), normal(normal) {}

float DistancePointPlane(const Point& pnt, const Plane& pln) {
    auto c2p = pnt - pln.center;
    return mathpls::dot(pln.normal, c2p);
}

bool IntersectSpherePlane(const Sphere& sph, const Plane& pln) {
    auto dis = std::abs(DistancePointPlane(sph.center, pln));
    return LESS_EQUAL(dis, sph.radius);
}

bool IsSphereAbovePlane(const Sphere& sph, const Plane& pln) {
    auto dis = DistancePointPlane(sph.center, pln);
    return LESS(sph.radius, dis);
}

bool IsSphereBelowPlane(const Sphere& sph, const Plane& pln) {
    auto dis = DistancePointPlane(sph.center, pln);
    return LESS(sph.radius, -dis);
}

}
