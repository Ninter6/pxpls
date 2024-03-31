//
//  Collision.hpp
//  pxpls
//
//  Created by Ninter6 on 2023/7/14.
//

#pragma once

#include "mathpls.h"
#include "Transform.hpp"

#include <unordered_map>

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

namespace algo {
CollisionPoints FindSphereSphereCollisionPoints(const SphereCollider& a, const Transform& ta, const SphereCollider& b, const Transform& tb);

CollisionPoints FindSpherePlaneCollisionPoints(const SphereCollider& a, const Transform& ta, const PlaneCollider& b, const Transform& tb);

CollisionPoints FindPlaneSphereCollisionPoints(const PlaneCollider& a, const Transform& ta, const SphereCollider& b, const Transform& tb);

CollisionPoints FindBoxBoxCollisionPoints(const PlaneCollider& a, const Transform& ta, const PlaneCollider& b, const Transform& tb);
}

struct Collider {
    Collider() = default;
    virtual ~Collider() = default;
    
    virtual CollisionPoints TestCollision(const Transform& transform, const Collider& collider, const Transform& colliderTransForm) const = 0;
    
    virtual CollisionPoints TestCollision(const Transform& transform, const SphereCollider& collider, const Transform& colliderTransForm) const = 0;
    
    virtual CollisionPoints TestCollision(const Transform& transform, const PlaneCollider& collider, const Transform& colliderTransForm) const = 0;
};


struct SphereCollider : public Collider {
    SphereCollider() = default;
    SphereCollider(mathpls::vec3 center, float radius);
    
    mathpls::vec3 Center;
    float Radius;
    
    virtual CollisionPoints TestCollision(const Transform& transform, const Collider& collider, const Transform& colliderTransForm) const override;
    
    virtual CollisionPoints TestCollision(const Transform& transform, const SphereCollider& collider, const Transform& colliderTransForm) const override;
    
    virtual CollisionPoints TestCollision(const Transform& transform, const PlaneCollider& collider, const Transform& colliderTransForm) const override;
};

struct PlaneCollider : Collider {
    PlaneCollider() = default;
    PlaneCollider(mathpls::vec3 normal, mathpls::vec3 point);
    
    mathpls::vec3 Normal, Point;
    
    virtual CollisionPoints TestCollision(const Transform& transform, const Collider& collider, const Transform& colliderTransForm) const override;

    virtual CollisionPoints TestCollision(const Transform& transform, const SphereCollider& collider, const Transform& colliderTransForm) const override;

    virtual CollisionPoints TestCollision(const Transform& transform, const PlaneCollider& collider, const Transform& colliderTransForm) const override;
};

struct CollisionBody {
    CollisionBody();
    
    using id_t = uint64_t;
    using Map = std::unordered_map<id_t, CollisionBody*>;
    
    id_t id;
    Transform transform{};
    Collider* collider{nullptr};
    
    bool IsTrigger = false;
    bool IsKinematic = false;
    bool IsDynamic = false;
    
private:
    static id_t currentId;
    
};

struct Collision {
    CollisionBody* A;
    CollisionBody* B;
    CollisionPoints points;
};

}
