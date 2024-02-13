//
//  Collision2D.cpp
//  pxpls
//
//  Created by Ninter6 on 2023/12/29.
//

#include "Collision2D.hpp"

#include <algorithm>
#include <cassert>

namespace pxpls {

namespace algo {

mathpls::vec2 RotateVec(mathpls::vec2 vec, mathpls::angle_t angle) {
    mathpls::mat2 RotMat = mathpls::rotate(angle);
    return RotMat * vec;
}

/**
 * \param pa a point on line A
 * \param da the direction of A
 * \param pb a point on line B
 * \param db the direction of B
 */
mathpls::vec2 LineIntersection(mathpls::vec2 pa, mathpls::vec2 da,
                               mathpls::vec2 pb, mathpls::vec2 db) {
    // calcu the analytical formula for line A
    float ka, ba;
    ka = da.y / da.x;
    ba = pa.y - ka * pa.x;
    
    // calcu the analytical formula for line B
    float kb, bb;
    kb = db.y / db.x;
    bb = pb.y - kb * pb.x;
    
    return {(bb - ba) / (ka - kb),
            (bb - ba) / (ka - kb) * ka + ba};
}

CollisionPoints FindCircleCilcleCollisionPoints(const CircleCollider* a,
                                                const Transform2D* at,
                                                const CircleCollider* b,
                                                const Transform2D* bt) {
    auto aPos = a->Center + at->Position, bPos = b->Center + bt->Position;
    auto aR = a->Radius * at->Scale.x, bR = b->Radius * bt->Scale.x;
    
    auto aTob = bPos - aPos;
    auto distence = aTob.length();
    
    if (distence >= aR + bR || distence <= abs(aR - bR)) return {};
    
    aTob.normalize();
    
    CollisionPoints res{};
    
    res.HasCollision = true;
    if (distence >= std::max(aR, bR)) {
        res.A = aPos + aTob * aR;
        res.B = bPos - aTob * bR;
        res.Depth = aR + bR - distence;
        res.Normal = aTob;
    } else {
        if (aR > bR) {
            res.A = aPos + aTob * aR;
            res.B = bPos + aTob * bR;
        } else {
            res.A = aPos - aTob * aR;
            res.B = bPos - aTob * bR;
        }
        res.Depth = distence - std::abs(aR - bR);
        res.Normal = -aTob;
    }
    
    return res;
}

/**
 * \param a the line collider object
 * \param at the transform of object a
 * \param b the circle collider object
 * \param bt the transform of object b
 */
CollisionPoints FindLineCilcleCollisionPoints(const LineCollider* a,
                                              const Transform2D* at,
                                              const CircleCollider* b,
                                              const Transform2D* bt) {
    const auto aPos = a->Origin + at->Position, bPos = b->Center + bt->Position;
    const auto aVec = RotateVec(a->Vector, at->Rotation) * at->Scale;
    const auto bR = b->Radius * bt->Scale.x;
    const auto bR2 = bR * bR;
    
    // move b to the local space of a
    const auto bPosInLS = bPos - aPos;
    
    if (bPosInLS.length_squared() <= bR2 &&
        (bPosInLS - aVec).length_squared() <= bR2)
        return {}; // the line is inside the circle
    
    // determine whether the vertical foot is on the line segment
    auto foot = mathpls::project(bPosInLS, aVec);
    if (mathpls::dot(foot, aVec) < 0)
        foot = {0};
    else if (foot.length_squared() > aVec.length_squared())
        foot = aVec;
    
    const auto perp = bPosInLS - foot;
    auto depth = bR - perp.length();
    
    if (depth <= 0) return {}; // no collision happened
    
    CollisionPoints res{};
    
    res.HasCollision = true;
    res.A = foot + aPos;
    res.B = bPos - perp.normalized() * bR;
    res.Depth = depth;
    res.Normal = -perp.normalized();
    
    return res;
}

CollisionPoints FindLineLineCollisionPoints(const LineCollider* a,
                                            const Transform2D* at,
                                            const LineCollider* b,
                                            const Transform2D* bt) {
    const auto aO = a->Origin + at->Position, bO = b->Origin + bt->Position;
    const auto aVec = RotateVec(a->Vector, at->Rotation) * at->Scale, bVec = RotateVec(b->Vector, bt->Rotation) * bt->Scale;
    
    // find the intersection of the 2 lines
    auto intersection = LineIntersection(aO, aVec, aO, bVec);
    auto aToi = intersection - aO;
    auto bToi = intersection - bO;
    
    if (mathpls::dot(aVec, bToi) < 0 || aVec.length_squared() < aToi.length_squared() ||
        mathpls::dot(bVec, bToi) < 0 || bVec.length_squared() < bToi.length_squared())
        return {}; // no collision happened
    
    mathpls::vec2 closest;
    if (bToi.length() < bVec.length() / 2) closest = bToi;
    else closest = bVec - bToi;
    
    auto perp = mathpls::perpendicular(closest, aVec);
    
    CollisionPoints res;
    
    res.HasCollision = true;
    res.A = intersection;
    res.B = intersection;
    res.Depth = perp.length();
    res.Normal = perp.normalized();
    
    return res;
}

CollisionPoints FindLineAabbCollisionPoints(const LineCollider* a,
                                            const Transform2D* at,
                                            const AabbColloder* b,
                                            const Transform2D* bt) {
    // TODO
    return {};
}

CollisionPoints FindCircleAabbCollisionPoints(const CircleCollider* a,
                                              const Transform2D* at,
                                              const AabbColloder* b,
                                              const Transform2D* bt) {
    CollisionPoints res{};
    for (const auto& i : b->GetSides()) {
        auto cp = FindLineCilcleCollisionPoints(&i, bt, a, at);
        if (cp.HasCollision && cp.Depth > res.Depth)
            res = cp;
    }
    return res;
}

CollisionPoints FindAabbAabbCollisionPoints(const AabbColloder* a,
                                            const Transform2D* at,
                                            const AabbColloder* b,
                                            const Transform2D* bt) {
    // TODO
    return {};
}

}

CollisionPoints CollisionPoints::Swaped() const {
    CollisionPoints r = *this;
    std::swap(r.A, r.B);
    r.Normal *= -1;
    return r;
}

bool Bounds2D::IsOverlapping(const Bounds2D& o) const {
    return (max.x > o.min.x && min.x < o.max.x &&
            max.y > o.min.y && min.y < o.max.y);
}

CircleCollider::CircleCollider(mathpls::vec2 center, float radius)
: Center(center), Radius(radius) {}

CollisionPoints CircleCollider::TestCollision(const Transform2D *transform,
                                              const Collider *collider,
                                              const Transform2D *colTransform) const {
    return collider->TestCollision(colTransform, this, transform).Swaped();
}

CollisionPoints CircleCollider::TestCollision(const Transform2D *transform,
                                              const CircleCollider *collider,
                                              const Transform2D *colTransform) const {
    return algo::FindCircleCilcleCollisionPoints(this, transform, collider, colTransform);
}

CollisionPoints CircleCollider::TestCollision(const Transform2D *transform,
                                              const LineCollider *collider,
                                              const Transform2D *colTransform) const {
    return algo::FindLineCilcleCollisionPoints(collider, colTransform, this, transform).Swaped();
}

CollisionPoints CircleCollider::TestCollision(const Transform2D *transform,
                                              const AabbColloder *collider,
                                              const Transform2D *colTransform) const {
    return algo::FindCircleAabbCollisionPoints(this, transform, collider, colTransform);
}

Bounds2D CircleCollider::GetBounds(const Transform2D* transform) const {
    auto center = Center + transform->Position;
    float R = std::abs(Radius * transform->Scale.x);
    
    return {center - R, center + R};
}

LineCollider::LineCollider(mathpls::vec2 origin, mathpls::vec2 vec)
: Origin(origin), Vector(vec) {}

CollisionPoints LineCollider::TestCollision(const Transform2D *transform,
                                            const Collider *collider,
                                            const Transform2D *colTransform) const {
    return collider->TestCollision(colTransform, this, transform).Swaped();
}

CollisionPoints LineCollider::TestCollision(const Transform2D *transform,
                                            const CircleCollider *collider,
                                            const Transform2D *colTransform) const {
    return algo::FindLineCilcleCollisionPoints(this, transform, collider, colTransform);
}

CollisionPoints LineCollider::TestCollision(const Transform2D *transform,
                                            const LineCollider *collider,
                                            const Transform2D *colTransform) const {
    return algo::FindLineLineCollisionPoints(this, transform, collider, colTransform);
}

CollisionPoints LineCollider::TestCollision(const Transform2D *transform,
                                            const AabbColloder *collider,
                                            const Transform2D *colTransform) const {
    return algo::FindLineAabbCollisionPoints(this, transform, collider, colTransform);
}

Bounds2D LineCollider::GetBounds(const Transform2D *transform) const {
    auto vec = algo::RotateVec(Vector, transform->Rotation) * transform->Scale;
    auto ori = Origin + transform->Position;
    auto end = ori + vec;
    
    return {
        {std::min(ori.x, end.x), std::min(ori.y, end.y)},
        {std::max(ori.x, end.x), std::max(ori.y, end.y)}
    };
}

AabbColloder::AabbColloder(mathpls::vec2 pos, mathpls::vec2 ext)
: Position(pos), Extent(ext) {
    assert(ext.x > 0 && ext.y > 0 && "invalid extent!");
}

CollisionPoints AabbColloder::TestCollision(const Transform2D *transform,
                                            const Collider *collider,
                                            const Transform2D *colTransform) const {
    return collider->TestCollision(colTransform, this, transform).Swaped();
}

CollisionPoints AabbColloder::TestCollision(const Transform2D *transform,
                                            const CircleCollider *collider,
                                            const Transform2D *colTransform) const {
    return algo::FindCircleAabbCollisionPoints(collider, colTransform, this, transform).Swaped();
}

CollisionPoints AabbColloder::TestCollision(const Transform2D *transform,
                                            const LineCollider *collider,
                                            const Transform2D *colTransform) const {
    return algo::FindLineAabbCollisionPoints(collider, colTransform, this, transform).Swaped();
}

CollisionPoints AabbColloder::TestCollision(const Transform2D *transform,
                                            const AabbColloder *collider,
                                            const Transform2D *colTransform) const {
    return algo::FindAabbAabbCollisionPoints(this, transform, collider, colTransform);
}

Bounds2D AabbColloder::GetBounds(const Transform2D *transform) const {
    auto min = Position + transform->Position;
    auto max = min + Extent * transform->Scale;
    
    return {min, max};
}

std::vector<LineCollider> AabbColloder::GetSides(const Transform2D *transform) const {
    const auto& [min, max] = 
        transform ? GetBounds(transform) : Bounds2D{Position, Position + Extent};
    
    return {
        {min, {Extent.x, 0}},               // bottom
        {{max.x, min.y}, {0, Extent.y}},    // right
        {max, {-Extent.x, 0}},              // top
        {{min.x, max.y}, {0, -Extent.y}}    // left
    };
}

CollisionBody::id_t CollisionBody::currentId = 0;

CollisionBody::CollisionBody() {
    id = currentId++;
}

mathpls::vec2& CollisionBody::Position() {
    return Transform.Position;
}
const mathpls::vec2& CollisionBody::Position() const {
    return Transform.Position;
}

Bounds2D CollisionBody::GetBounds() const {
    if (Collider) {
        return Collider->GetBounds(&Transform);
    } else {
        return {};
    }
}

void CollisionBody::SetCollisionCallback(const CollisionCallback& callback) {
    m_OnCollisions = callback;
}

void CollisionBody::OnCollision(const Collision& collision, float deltaTime) const {
    if (m_OnCollisions) {
        m_OnCollisions(collision, deltaTime);
    }
}

}
