//
//  Geometry.cpp
//  pxpls
//
//  Created by Ninter6 on 2024/2/19.
//

#include "Geometry.hpp"

#include <cmath>
#include <limits>
#include <algorithm>

#define EPSILON std::numeric_limits<float>::epsilon()
#define LESS(a, b) b - a > EPSILON
#define LESS_EQUAL(a, b) a - b <= EPSILON
#define EQUAL(a, b) std::abs(a - b) <= EPSILON

namespace pxpls {

Sphere::Sphere(const Point& center, float radius) : center(center), radius(radius) {}

Plane::Plane(const mathpls::vec4& v) : normal(v), D(v[3]) {}

Plane::Plane(const mathpls::vec3& n, float d) : normal(n), D(d) {}

Plane::Plane(const Point& p0, const mathpls::vec3& normal)
: D(-mathpls::dot(p0, normal)), normal(normal) {}

Point Plane::P() const {
    return normal * D;
}

Bounds::Bounds(const Point& min, const Point& max) : min(min), max(max) {}

Point Bounds::center() const {
    return (min + max) * .5f;
}

std::array<Point, 8> Bounds::allVertices() const {
    return {
        max,
        {min.x, max.y, max.z},
        {min.x, max.y, min.z},
        {max.x, max.y, min.z},
        {max.x, min.y, min.z},
        {max.x, min.y, max.z},
        {min.x, min.y, max.z},
        min
    };
}

float DistancePointPlane(const Point& pnt, const Plane& pln) {
    return mathpls::dot(pln.normal, pnt) + pln.D;
}

Point ProjectPointPlane(const Point& pnt, const Plane& pln) {
    return pnt - DistancePointPlane(pnt, pln) * pln.normal;
}

Point IntersectPlanes(const Plane& A, const Plane& B, const Plane& C) {
    auto M = mathpls::mat3{
         A.normal,
         B.normal,
         C.normal
    }.transposed();
    auto b = -mathpls::vec3{A.D, B.D, C.D};
    return mathpls::inverse(M) * b;
}

bool IsPointInSphere(const Point& pnt, const Sphere& sph) {
    return LESS(mathpls::distance_quared(pnt, sph.center), sph.radius * sph.radius);
}

bool IsPointOnSphere(const Point& pnt, const Sphere& sph) {
    return EQUAL(mathpls::distance_quared(pnt, sph.center), sph.radius * sph.radius);
}

bool IsPointOutSphere(const Point& pnt, const Sphere& sph) {
    return LESS(sph.radius * sph.radius, mathpls::distance_quared(pnt, sph.center));
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

bool IntersectSphereSphere(const Sphere& s1, const Sphere& s2) {
    return LESS_EQUAL(mathpls::distance_quared(s1.center, s2.center),
                      (s1.radius + s2.radius) * (s1.radius + s2.radius));
}

bool IntersectBoundsPlane(const Bounds& bnd, const Plane& pln) {
    bool pos = false, neg = false;
    for (const auto& i : bnd.allVertices()) {
        auto dis = DistancePointPlane(i, pln);
        
        if (LESS(dis, 0))
            neg = true;
        else if (LESS(0, dis))
            pos = true;
        else
            return true; // point on plane
        
        if (pos && neg)
            return true;
    }
    return false;
}

bool IsBoundsAbovePlane(const Bounds& bnd, const Plane& pln) {
    auto v = bnd.allVertices();
    return std::all_of(v.begin(), v.end(), [&](auto&&i){
        return LESS(0, DistancePointPlane(i, pln));
    });
}

bool IsBoundsBelowPlane(const Bounds& bnd, const Plane& pln) {
    auto v = bnd.allVertices();
    return std::all_of(v.begin(), v.end(), [&](auto&&i){
        return LESS(DistancePointPlane(i, pln), 0);
    });
}

}
