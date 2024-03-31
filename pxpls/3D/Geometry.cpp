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
#include <numeric>

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

Sphere BoundingSphereFromPoints(std::span<Point> points) {
    if (points.size() == 0) return {};
    
    pxpls::Sphere sphere{points[0], 0};
    pxpls::Point sp[4];

    auto make_sph = [&](int ns) {
        switch (ns) {
        case 2: {
            sphere.center = (sp[0] + sp[1]) / 2;
            sphere.radius = mathpls::distance(sp[0], sp[1]) / 2;
            break;
        }
        case 3: {
            auto e0 = sp[1] - sp[0], e1 = sp[2] - sp[0];
            auto a = mathpls::dot(e0, e0), b = mathpls::dot(e0, e1), c = mathpls::dot(e1, e1);
            auto d = a * c - b * b;
            if (std::abs(d) > 1e-3f) {
                auto s = (a - b)*c / (2 * d), t = (c - b)*a / (2 * d);
                sphere.center = sp[0] + s * e0 + t * e1;
                sphere.radius = (sp[0] - sphere.center).length();
            }
            break;
        }
        case 4: {
            auto v1 = sp[1] - sp[0], v2 = sp[2] - sp[0], v3 = sp[3] - sp[0];
            auto V = mathpls::dot(v1, mathpls::cross(v2, v3));
            // Check that the three points are not on the same plane.
            if (std::abs(V) > 1e-3f) {
                V *= 2.0;
                auto L1 = v1.length_squared(), L2 = v2.length_squared(), L3 = v3.length_squared();
                sphere.center.x = (sp[0].x + ((v2.y*v3.z - v3.y*v2.z)*L1 - (v1.y*v3.z - v3.y*v1.z)*L2 + (v1.y*v2.z - v2.y*v1.z)*L3) / V);
                sphere.center.y = (sp[0].y + (-(v2.x*v3.z - v3.x*v2.z)*L1 + (v1.x*v3.z - v3.x*v1.z)*L2 - (v1.x*v2.z - v2.x*v1.z)*L3) / V);
                sphere.center.z = (sp[0].z + ((v2.x*v3.y - v3.x*v2.y)*L1 - (v1.x*v3.y - v3.x*v1.y)*L2 + (v1.x*v2.y - v2.x*v1.y)*L3) / V);
                sphere.radius = (sphere.center - sp[0]).length();
            }
            break;
        }
        default:
            break;
        }
    };

    for (uint32_t i = 1; i < points.size(); i++) {
        if (!pxpls::IsPointOutSphere(points[i], sphere)) continue;
        sp[0] = points[i];
        for (uint32_t j = 0; j < i; j++) {
            if (!pxpls::IsPointOutSphere(points[j], sphere)) continue;
            sp[1] = points[j];
            make_sph(2);
            for (uint32_t k = 0; k < j; k++) {
                if (!pxpls::IsPointOutSphere(points[k], sphere)) continue;
                sp[2] = points[k];
                make_sph(3);
                for (uint32_t l = 0; l < k; l++) {
                    if (!pxpls::IsPointOutSphere(points[l], sphere)) continue;
                    sp[3] = points[l];
                    make_sph(4);
                }
            }
        }
    }

    return sphere;
}

pxpls::Sphere BoundingSphereFromPointsFast(std::span<pxpls::Point> points) {
    if (points.size() == 0) return {};
    
    pxpls::Sphere sphere{};
    
    sphere.center = std::reduce(points.begin(), points.end());
    sphere.center /= points.size();
    sphere.radius = mathpls::distance(*std::max_element(points.begin(), points.end(), [&](auto&& a, auto&& b){
        return mathpls::distance_quared(a, sphere.center) < mathpls::distance_quared(b, sphere.center);
    }), sphere.center);
    
    return sphere;
}

}
