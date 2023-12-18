//
//  Collider.hpp
//  pxpls
//
//  Created by Ninter6 on 2023/7/14.
//

#pragma once

#include <iostream>
#include "mathpls.h"
#include "Transform.hpp"

namespace pxpls {

struct CollisionPoints {
    mathpls::vec3 A;
    mathpls::vec3 B;
    mathpls::vec3 Normal;
    float Depth;
    bool HasCollision;
};

struct Collider;
struct SphereCollider;
struct PlaneCollider;

struct Collider {
    Collider() = default;
    virtual ~Collider() = default;
    
    virtual CollisionPoints TestCollision(const TransForm& transform, const Collider& collider, const TransForm& colliderTransForm) const = 0;
    
    virtual CollisionPoints TestCollision(const TransForm& transform, const SphereCollider& collider, const TransForm& colliderTransForm) const = 0;
    
    virtual CollisionPoints TestCollision(const TransForm& transform, const PlaneCollider& collider, const TransForm& colliderTransForm) const = 0;
};


struct SphereCollider : public Collider {
    SphereCollider() = default;
    SphereCollider(mathpls::vec3 center, float radius);
    
    mathpls::vec3 Center;
    float Radius;
    
    virtual CollisionPoints TestCollision(const TransForm& transform, const Collider& collider, const TransForm& colliderTransForm) const override;
    
    virtual CollisionPoints TestCollision(const TransForm& transform, const SphereCollider& collider, const TransForm& colliderTransForm) const override;
    
    virtual CollisionPoints TestCollision(const TransForm& transform, const PlaneCollider& collider, const TransForm& colliderTransForm) const override;
};

struct PlaneCollider : Collider {
    PlaneCollider() = default;
    PlaneCollider(mathpls::vec3 normal, mathpls::vec3 point);
    
    mathpls::vec3 Normal, Point;
    
    virtual CollisionPoints TestCollision(const TransForm& transform, const Collider& collider, const TransForm& colliderTransForm) const override;

    virtual CollisionPoints TestCollision(const TransForm& transform, const SphereCollider& collider, const TransForm& colliderTransForm) const override;

    virtual CollisionPoints TestCollision(const TransForm& transform, const PlaneCollider& collider, const TransForm& colliderTransForm) const override;
};


namespace algo {
CollisionPoints FindSphereSphereCollisionPoints(const SphereCollider& a, const TransForm& ta, const SphereCollider& b, const TransForm& tb);

CollisionPoints FindSpherePlaneCollisionPoints(const SphereCollider& a, const TransForm& ta, const PlaneCollider& b, const TransForm& tb);

CollisionPoints FindPlaneSphereCollisionPoints(const PlaneCollider& a, const TransForm& ta, const SphereCollider& b, const TransForm& tb);

CollisionPoints FindBoxBoxCollisionPoints(const PlaneCollider& a, const TransForm& ta, const PlaneCollider& b, const TransForm& tb);
}

}
