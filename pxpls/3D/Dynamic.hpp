//
//  Dynamic.hpp
//  pxpls
//
//  Created by Ninter6 on 2023/8/5.
//

#pragma once

#include "Collision.hpp"

namespace pxpls {

struct Rigidbody : CollisionBody {
    Rigidbody();
    
    float InvMass() const;
    void ApplyForce(const mathpls::vec3& force);
    
    float mass = 1.f;
    
    /**
     * \brief Set position and lastPosition
     * \note position and lastPosition will be the same
     */
    void SetPosition(const mathpls::vec3& pos);
    
    /**
     * \brief Apply an offset to the position
     * \note lastPosition wont change
     */
    void Translate(const mathpls::vec3& offset);
    
    mathpls::vec3 lastPostion{};
    mathpls::vec3 acceleration{0};
    
    bool takeGravity;
};

}
