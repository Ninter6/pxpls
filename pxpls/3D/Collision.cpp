//
//  Collider.cpp
//  pxpls
//
//  Created by Ninter6 on 2023/7/14.
//

#include "Collision.hpp"

#include <cmath>
#include <limits>

namespace pxpls {

namespace algo {

CollisionPoints FindSphereSphereCollisionPoints(const SphereCollider& a, const Transform& ta, const SphereCollider& b, const Transform& tb){
    //        const auto& oa = a.Center * ta.Scale + ta.Position, ob = b.Center * tb.Scale + tb.Position;
    //        // 先以a的圆心为原点
    //        // 将a变为单位圆
    //        const auto& tob = (ob - oa) / ta.Scale / a.Radius;
    //        const auto& tA = tob.normalize() * ta.Scale * a.Radius + oa;
    //        // 再以b的圆心为原点
    //        // 将b变为单位圆
    //        const auto& toa = (oa - ob) / tb.Scale / b.Radius;
    //        const auto& tB = toa.normalize() * tb.Scale * b.Radius + ob;
    //
    //        CollisionPoints rz;
    //
    //        rz.A = tA;
    //        rz.B = tB;
    //        rz.Depth = (tA - oa).length() + (tB - ob).length() - (ob - oa).length();
    //        rz.Normal = (tB - tA).normalize();
    //        rz.HasCollision = rz.Depth >= 0;
    //
    //        return rz;
    
    const auto& oa = a.Center + ta.Position, ob = b.Center + tb.Position;
    const auto& ra = a.Radius * std::abs(ta.Scale.x), rb = b.Radius * std::abs(tb.Scale.x);
    
    auto b2a = oa - ob;
    auto len = std::sqrtf(b2a.length_squared());
    b2a.normalize();
    
    CollisionPoints rz;
    
    rz.A = b2a *-ra + oa;
    rz.B = b2a * rb + ob;
    rz.Depth = ra + rb - len;
    rz.Normal = b2a;
    rz.HasCollision = rz.Depth > 0;
    
    return rz;
}

CollisionPoints FindSpherePlaneCollisionPoints(const SphereCollider& a, const Transform& ta, const PlaneCollider& b, const Transform& tb){
    // NP式平面方程
    // d(P) = dot(N, P - P0)
    //
    // d(P) == 0    P在平面上
    // d(P) >  0    P在平面上方
    // d(P) <  0    P在平面下方
    //
    // P到平面距离    |d(P)|
    
    const auto& oa = a.Center + ta.Position, ob = b.plane.P() + tb.Position;
    const auto& bn = mathpls::mat3(mathpls::rotate(tb.Rotation)) * b.plane.normal;
    
    // 计算距离
    const auto ds = mathpls::dot(bn, oa - ob);
    
    CollisionPoints rz;
    
    rz.A = oa + (ds < 0 ? 1.f : -1.f) * bn * a.Radius * ta.Scale.x;
    rz.B = oa - bn * ds;
    rz.Depth = a.Radius - abs(ds);
    rz.Normal = (rz.B - rz.A).normalized();
    rz.HasCollision = rz.Depth > 0;
    
    return rz;
}

CollisionPoints FindPlaneSphereCollisionPoints(const PlaneCollider& a, const Transform& ta, const SphereCollider& b, const Transform& tb){
    
    CollisionPoints rz = FindSpherePlaneCollisionPoints(b, tb, a, ta);
    
    rz.Normal *= -1;
    
    return rz;
}

CollisionPoints FindBoxBoxCollisionPoints(const PlaneCollider& a, const Transform& ta, const PlaneCollider& b, const Transform& tb){
    
    return {};
}

}

SphereCollider::SphereCollider(mathpls::vec3 center, float radius) : Center(center), Radius(radius) {}

CollisionPoints SphereCollider::TestCollision(const Transform& transform, const Collider& collider, const Transform& colliderTransForm) const {
    return collider.TestCollision(colliderTransForm, *this, transform);
}

CollisionPoints SphereCollider::TestCollision(const Transform &transform, const SphereCollider &collider, const Transform &colliderTransForm) const {
    return algo::FindSphereSphereCollisionPoints(*this, transform, collider, colliderTransForm);
}

CollisionPoints SphereCollider::TestCollision(const Transform &transform, const PlaneCollider &collider, const Transform &colliderTransForm) const {
    return algo::FindSpherePlaneCollisionPoints(*this, transform, collider, colliderTransForm);
}

Bounds SphereCollider::GetBounds(const Transform& transform) const {
    auto c = Center + transform.Position;
    auto r = Radius * std::abs(transform.Scale.x);
    
    return {
        c - r,
        c + r
    };
}

PlaneCollider::PlaneCollider(const Plane& plane) : plane(plane) {}
PlaneCollider::PlaneCollider(const mathpls::vec4& v) : plane(v) {}
PlaneCollider::PlaneCollider(const mathpls::vec3& normal, float D) : plane(normal, D) {}
PlaneCollider::PlaneCollider(const Point& p0, const mathpls::vec3& normal) : plane(p0, normal) {}

CollisionPoints PlaneCollider::TestCollision(const Transform &transform, const Collider &collider, const Transform &colliderTransForm) const {
    return collider.TestCollision(colliderTransForm, *this, transform);
}

CollisionPoints PlaneCollider::TestCollision(const Transform &transform, const SphereCollider &collider, const Transform &colliderTransForm) const {
    return algo::FindPlaneSphereCollisionPoints(*this, transform, collider, colliderTransForm);
}

CollisionPoints PlaneCollider::TestCollision(const Transform &transform, const PlaneCollider &collider, const Transform &colliderTransForm) const {
    return algo::FindBoxBoxCollisionPoints(*this, transform, collider, colliderTransForm);
}

Bounds PlaneCollider::GetBounds(const Transform& transform) const {
    const auto& o = plane.P() + transform.Position;
    const auto& n = mathpls::mat3(mathpls::rotate(transform.Rotation)) * plane.normal;
    
    Bounds res{};
    for (int i = 0; i < 3; i++) {
        if (n[i] > .99f) {
            res.min[i] = o[i];
            res.max[i] = o[i];
        } else {
            res.min[i] = std::numeric_limits<float>::lowest();
            res.max[i] = std::numeric_limits<float>::max();
        }
    }
    return res;
}

CollisionBody::id_t CollisionBody::currentId = 0;

CollisionBody::CollisionBody() {
    id = currentId++;
}

mathpls::vec3& CollisionBody::Position() {
    return transform.Position;
}

const mathpls::vec3& CollisionBody::Position() const {
    return transform.Position;
}

Bounds CollisionBody::GetBounds() const {
    return collider->GetBounds(transform);
}

void CollisionBody::OnCollision(const Collision& collision, float dt) const {
    if (callback)
        callback(collision, dt);
}

}
