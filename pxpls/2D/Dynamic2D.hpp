//
//  Dynamic2D.hpp
//  pxpls
//
//  Created by Ninter6 on 2023/12/30.
//

#pragma once

#include "Collision2D.hpp"

namespace pxpls {

class Rigidbody : public CollisionBody {
public:
    Rigidbody();
    
    float InvMass() const;
    
    mathpls::vec2& ApplyForce(mathpls::vec2 addedForce);
    
    mathpls::vec2 Force;
    mathpls::vec2 Velocity;

    float Mass{};
    bool TakesGravity{};

    float StaticFriction{};
    float DynamicFriction{};
    float Restitution{};
};

}
