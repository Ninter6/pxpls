//
//  Collision2D.hpp
//  pxpls
//
//  Created by Ninter6 on 2023/12/29.
//

#pragma once

#include "Transform2D.h"

#include <vector>
#include <functional>
#include <unordered_map>

namespace pxpls {

using Point2D = mathpls::vec2;

struct CollisionPoints {
    /**
     * \brief the deepest point of obj A inside obj B
     */
    Point2D A;
    
    /**
     * \brief the deepest point of obj B inside obj A
     */
    Point2D B;
    
    /**
     * \brief normalize(B - A)
     */
    mathpls::vec2 Normal;
    
    /**
     * \brief length(B - A)
     */
    float Depth;
    
    /**
     * \brief boolean indicating whether a collision happened.
     */
    bool HasCollision = false;
    
    /**
     * Swap A and B
     */
    CollisionPoints Swaped() const;
};

struct Bounds2D {
    Point2D min, max;
    
    Point2D center() const {return (min + max) / 2;}
    Point2D extent() const {return max - min;}
    
    bool IsOverlapping(const Bounds2D& o) const;
};

class CircleCollider;
class LineCollider;
class AabbColloder;

class Collider {
public:
    virtual ~Collider() = default;
    
    virtual CollisionPoints TestCollision(const Transform2D* transform,
                                          const Collider* collider,
                                          const Transform2D* colTransform) const = 0;
    
    virtual CollisionPoints TestCollision(const Transform2D* transform,
                                          const CircleCollider* collider,
                                          const Transform2D* colTransform) const = 0;
    
    virtual CollisionPoints TestCollision(const Transform2D* transform,
                                          const LineCollider* collider,
                                          const Transform2D* colTransform) const = 0;
    
    virtual CollisionPoints TestCollision(const Transform2D* transform,
                                          const AabbColloder* collider,
                                          const Transform2D* colTransform) const = 0;
    
    virtual Bounds2D GetBounds(const Transform2D* transform) const = 0;
};

class CircleCollider : public Collider {
public:
    CircleCollider() = default;
    CircleCollider(Point2D center, float radius);
    
    virtual CollisionPoints TestCollision(const Transform2D* transform,
                                          const Collider* collider,
                                          const Transform2D* colTransform) const override;
    
    virtual CollisionPoints TestCollision(const Transform2D* transform,
                                          const CircleCollider* collider,
                                          const Transform2D* colTransform) const override;
    
    virtual CollisionPoints TestCollision(const Transform2D* transform,
                                          const LineCollider* collider,
                                          const Transform2D* colTransform) const override;
    
    virtual CollisionPoints TestCollision(const Transform2D* transform,
                                          const AabbColloder* collider,
                                          const Transform2D* colTransform) const override;
    
    virtual Bounds2D GetBounds(const Transform2D* transform) const override;
    
    mathpls::vec2 Center;
    float Radius = 0;
};

class LineCollider : public Collider {
public:
    LineCollider() = default;
    LineCollider(Point2D origin, mathpls::vec2 vec);
    
    virtual CollisionPoints TestCollision(const Transform2D* transform,
                                          const Collider* collider,
                                          const Transform2D* colTransform) const override;
    
    virtual CollisionPoints TestCollision(const Transform2D* transform,
                                          const CircleCollider* collider,
                                          const Transform2D* colTransform) const override;
    
    virtual CollisionPoints TestCollision(const Transform2D* transform,
                                          const LineCollider* collider,
                                          const Transform2D* colTransform) const override;
    
    virtual CollisionPoints TestCollision(const Transform2D* transform,
                                          const AabbColloder* collider,
                                          const Transform2D* colTransform) const override;
    
    virtual Bounds2D GetBounds(const Transform2D* transform) const override;
    
    mathpls::vec2 Origin, Vector;
};

class AabbColloder : public Collider {
    AabbColloder() = default;
    AabbColloder(Point2D pos, mathpls::vec2 ext);
    
    virtual CollisionPoints TestCollision(const Transform2D* transform,
                                          const Collider* collider,
                                          const Transform2D* colTransform) const override;
    
    virtual CollisionPoints TestCollision(const Transform2D* transform,
                                          const CircleCollider* collider,
                                          const Transform2D* colTransform) const override;
    
    virtual CollisionPoints TestCollision(const Transform2D* transform,
                                          const LineCollider* collider,
                                          const Transform2D* colTransform) const override;
    
    virtual CollisionPoints TestCollision(const Transform2D* transform,
                                          const AabbColloder* collider,
                                          const Transform2D* colTransform) const override;
    
    virtual Bounds2D GetBounds(const Transform2D* transform) const override;
    
    std::vector<LineCollider> GetSides() const;
    
    mathpls::vec2 Position, Extent;
};

class CollisionBody;

struct Collision {
    CollisionBody* A;
    CollisionBody* B;
    CollisionPoints collisionPoints;
};

using CollisionCallback = std::function<void(const Collision&, float)>;

class CollisionBody {
public:
    CollisionBody();
    
    /**
     * \brief Sets the collision callback function.
     * \param callback The callback function.
     */
    void SetCollisionCallback(const CollisionCallback& callback);
    
    /**
     * \brief Triggers the collision callback function.
     * \param collision Object representing the collision.
     * \param deltaTime The time elapsed since the last frame.
     */
    void OnCollision(const Collision& collision, float deltaTime) const;
    
    /**
     * \brief Gets the position of the body in the world.
     * \return The position of the body in the world.
     */
    mathpls::vec2& Position();
    const mathpls::vec2& Position() const;
    
    /**
     * \brief Get the bounds of the body.
     * \note If the body has no collider, it will return a empty bounds.
     */
    Bounds2D GetBounds() const;
    
    using id_t = uint64_t;
    using Map = std::unordered_map<id_t, CollisionBody*>;
    
    id_t id;
    Transform2D Transform{};
    Collider* Collider{nullptr};
    
    bool IsTrigger = false;
    bool IsKinematic = false;
    bool IsDynamic = false;
    
    std::function<void(const Collision&, float)> m_OnCollisions{nullptr};
    
private:
    static id_t currentId;
    
};

}
