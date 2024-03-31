//
//  RigidBody.hpp
//  pxpls
//
//  Created by Ninter6 on 2023/8/5.
//

#pragma once

#include "Collision.hpp"

namespace pxpls {

struct RigidBody : CollisionBody {
    RigidBody();
    
    float InvMass() const;
    void ApplyForce(const mathpls::vec3& force);
    
    float mass = 1.f;
    
    mathpls::vec3 lastPostion{};
    mathpls::vec3 velocity{};
    mathpls::vec3 acceleration{0};
    
    bool takeGravity = false;
};

}
