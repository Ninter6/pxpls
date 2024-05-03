//
//  Dynamic2D.hpp
//  pxpls
//
//  Created by Ninter6 on 2023/12/30.
//

#pragma once

#include "Collision2D.hpp"

namespace pxpls {

class Rigidbody2D : public CollisionBody2D {
public:
    Rigidbody2D();
    
    float InvMass() const;
    
    mathpls::vec2& ApplyForce(mathpls::vec2 addedForce);
    
    /**
     * \brief Set new position and change last position
     * \result The last position
     */
    mathpls::vec2 SetPostion(const mathpls::vec2& pos);
    
    mathpls::vec2 Force;
    mathpls::vec2 Velocity;
    mathpls::vec2 LastPosition;

    float Mass{};
    bool TakesGravity{};

    float StaticFriction{};
    float DynamicFriction{};
    float Restitution{};
};

}
