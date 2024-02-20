//
//  Geometry.hpp
//  pxpls
//
//  Created by Ninter6 on 2024/2/19.
//

#pragma once

#include "mathpls.h"
#include <array>

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
    Plane(float distanceFromOrigin, const mathpls::vec3& normal);
    Plane(const Point& center, const mathpls::vec3& normal);
    
    Point P() const;
    
    float d2o; // distance from origin
    mathpls::vec3 normal;
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
