//
//  Geometry.hpp
//  pxpls
//
//  Created by Ninter6 on 2024/2/19.
//

#pragma once

#include "mathpls.h"

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
    Plane(const Point& center, const mathpls::vec3& normal);
    
    Point center;
    mathpls::vec3 normal;
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

}
