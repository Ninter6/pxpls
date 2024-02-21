//
//  Geometry.hpp
//  pxpls
//
//  Created by Ninter6 on 2024/2/19.
//

#pragma once

#include "mathpls.h"
#include <array>
#include <span>

namespace pxpls {

using Point = mathpls::vec3;

struct Sphere {
    Sphere() = default;
    Sphere(const Point& center, float radius);
    
    Point center;
    float radius;
};

struct Plane {
    Plane() = default;
    Plane(const mathpls::vec4& v);
    Plane(const mathpls::vec3& normal, float D);
    Plane(const Point& p0, const mathpls::vec3& normal);
    
    Point P() const;
    
    mathpls::vec3 normal;
    float D;
};

struct Bounds {
    Bounds() = default;
    Bounds(const Point& min, const Point& max);
    
    Point min;
    Point max;
    
    Point center() const;
    std::array<Point, 8> allVertices() const;
};

float DistancePointPlane(const Point& pnt, const Plane& pln);

/**
 * \brief test if distance of the two is less than the radius of sphere
 */
bool IsPointInSphere(const Point& pnt, const Sphere& sph);

/**
 * \brief test if distance of the two is equal to the radius of sphere
 */
bool IsPointOnSphere(const Point& pnt, const Sphere& sph);

/**
 * \brief test if distance of the two is larger than the radius of sphere
 */
bool IsPointOutSphere(const Point& pnt, const Sphere& sph);

/**
 * \brief test if absolute distance from sphere to plane is less than radius of sphere
 */
bool IntersectSpherePlane(const Sphere& sph, const Plane& pln);

/**
 * \brief test if the two dont intersect and distance from sphere to plane is positive
 */
bool IsSphereAbovePlane(const Sphere& sph, const Plane& pln);

/**
 * \brief test if the two dont intersect and distance from sphere to plane is negative
 */
bool IsSphereBelowPlane(const Sphere& sph, const Plane& pln);

/**
 * \brief test if distance between the two is less than the sum of their radius
 */
bool IntersectSphereSphere(const Sphere& s1, const Sphere& s2);

/**
 * \brief test if there are any vertices of bounds on both sides of plane
 */
bool IntersectBoundsPlane(const Bounds& bnd, const Plane& pln);

/**
 *  \brief test if all distance from each vertex to plane are positive
 */
bool IsBoundsAbovePlane(const Bounds& bnd, const Plane& pln);

/**
 * \brief test if all distance from each vertex to plane are negative
 */
bool IsBoundsBelowPlane(const Bounds& bnd, const Plane& pln);

}
