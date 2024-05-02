//
//  Collision.hpp
//  pxpls
//
//  Created by Ninter6 on 2023/7/14.
//

#pragma once

#include "Transform.h"
#include "Geometry.hpp"

#include <unordered_map>
#include <functional>

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
    
    virtual Bounds GetBounds(const Transform& transform) const = 0;
};


struct SphereCollider : public Collider {
    SphereCollider() = default;
    SphereCollider(mathpls::vec3 center, float radius);
    
    mathpls::vec3 Center;
    float Radius;
    
    virtual CollisionPoints TestCollision(const Transform& transform, const Collider& collider, const Transform& colliderTransForm) const override;
    
    virtual CollisionPoints TestCollision(const Transform& transform, const SphereCollider& collider, const Transform& colliderTransForm) const override;
    
    virtual CollisionPoints TestCollision(const Transform& transform, const PlaneCollider& collider, const Transform& colliderTransForm) const override;
    
    virtual Bounds GetBounds(const Transform& transform) const override;
};

struct PlaneCollider : Collider {
    PlaneCollider() = default;
    PlaneCollider(const Plane& plane);
    PlaneCollider(const mathpls::vec4& v);
    PlaneCollider(const mathpls::vec3& normal, float D);
    PlaneCollider(const Point& p0, const mathpls::vec3& normal);
    
    Plane plane;
    
    virtual CollisionPoints TestCollision(const Transform& transform, const Collider& collider, const Transform& colliderTransForm) const override;

    virtual CollisionPoints TestCollision(const Transform& transform, const SphereCollider& collider, const Transform& colliderTransForm) const override;

    virtual CollisionPoints TestCollision(const Transform& transform, const PlaneCollider& collider, const Transform& colliderTransForm) const override;
    
    virtual Bounds GetBounds(const Transform& transform) const override;
};

struct Collision;
using CollisionCallback = std::function<void(const Collision&, float)>;

struct CollisionBody {
    CollisionBody();
    
    using id_t = uint64_t;
    using Map = std::unordered_map<id_t, CollisionBody*>;
    
    id_t id;
    Transform transform{};
    Collider* collider{nullptr};
    
    CollisionCallback callback{nullptr};
    
    bool IsTrigger = false;
    bool IsKinematic = false;
    bool IsDynamic = false;
    
    mathpls::vec3& Position();
    const mathpls::vec3& Position() const;
    Bounds GetBounds() const;
    void OnCollision(const Collision& collision, float dt) const;
    
private:
    static id_t currentId;
    
};

struct Collision {
    CollisionBody* A;
    CollisionBody* B;
    CollisionPoints points;
};

using CollisionPair = std::pair<CollisionBody::id_t, CollisionBody::id_t>;

}

template <>
struct std::hash<pxpls::CollisionPair> {
    size_t operator()(const pxpls::CollisionPair& p) const {
        return std::hash<size_t>{}(p.first) ^ std::hash<size_t>{}(p.second) ^ 114514;
    }
};
