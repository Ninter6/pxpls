//
//  Collider.cpp
//  pxpls
//
//  Created by Ninter6 on 2023/7/14.
//

#include "Collider.hpp"

namespace pxpls {

SphereCollider::SphereCollider(mathpls::vec3 center, float radius) : Center(center), Radius(radius) {}

CollisionPoints SphereCollider::TestCollision(const TransForm& transform, const Collider& collider, const TransForm& colliderTransForm) const {
    return collider.TestCollision(colliderTransForm, *this, transform);
}

CollisionPoints SphereCollider::TestCollision(const TransForm &transform, const SphereCollider &collider, const TransForm &colliderTransForm) const {
    return algo::FindSphereSphereCollisionPoints(*this, transform, collider, colliderTransForm);
}

CollisionPoints SphereCollider::TestCollision(const TransForm &transform, const PlaneCollider &collider, const TransForm &colliderTransForm) const {
    return algo::FindSpherePlaneCollisionPoints(*this, transform, collider, colliderTransForm);
}

PlaneCollider::PlaneCollider(mathpls::vec3 normal, mathpls::vec3 point) : Normal(normal), Point(point) {}

CollisionPoints PlaneCollider::TestCollision(const TransForm &transform, const Collider &collider, const TransForm &colliderTransForm) const {
    return collider.TestCollision(colliderTransForm, *this, transform);
}

CollisionPoints PlaneCollider::TestCollision(const TransForm &transform, const SphereCollider &collider, const TransForm &colliderTransForm) const {
    return algo::FindPlaneSphereCollisionPoints(*this, transform, collider, colliderTransForm);
}

CollisionPoints PlaneCollider::TestCollision(const TransForm &transform, const PlaneCollider &collider, const TransForm &colliderTransForm) const {
    return algo::FindBoxBoxCollisionPoints(*this, transform, collider, colliderTransForm);
}

    namespace algo {
    
    CollisionPoints FindSphereSphereCollisionPoints(const SphereCollider& a, const TransForm& ta, const SphereCollider& b, const TransForm& tb){
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
        
        const auto& oa = a.Center * ta.Scale + ta.Position, ob = b.Center * tb.Scale + tb.Position;
        
        CollisionPoints rz;
        
        rz.A = (ob - oa).normalized() * a.Radius + oa;
        rz.B = (oa - ob).normalized() * b.Radius + ob;
        rz.Depth = (rz.B - rz.A).length();
        rz.Normal = (rz.B - rz.A).normalized();
        rz.HasCollision = (ob - oa).length() <= a.Radius + b.Radius;
        
        return rz;
    }
    
    CollisionPoints FindSpherePlaneCollisionPoints(const SphereCollider& a, const TransForm& ta, const PlaneCollider& b, const TransForm& tb){
        // NP式平面方程
        // d(P) = dot(N, P - P0)
        //
        // d(P) == 0    P在平面上
        // d(P) >  0    P在平面上方
        // d(P) <  0    P在平面下方
        //
        // P到平面距离    |d(P)|
        
        const auto& oa = a.Center + ta.Position, ob = b.Point + tb.Position;
        const auto& bn = (mathpls::mat3(mathpls::rotate(tb.Rotation)) * b.Normal).normalized();
        
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
    
    CollisionPoints FindPlaneSphereCollisionPoints(const PlaneCollider& a, const TransForm& ta, const SphereCollider& b, const TransForm& tb){
        
        CollisionPoints rz = FindSpherePlaneCollisionPoints(b, tb, a, ta);
        
        rz.Normal *= -1;
        
        return rz;
    }
    
    CollisionPoints FindBoxBoxCollisionPoints(const PlaneCollider& a, const TransForm& ta, const PlaneCollider& b, const TransForm& tb){
        
        return {};
    }
    
    }

}