//
//  Dynamic.cpp
//  pxpls
//
//  Created by Ninter6 on 2023/8/5.
//

#include "Dynamic.hpp"

#include <cmath>
#include <limits>

namespace pxpls {

Rigidbody::Rigidbody() : mass(1), takeGravity(true) {
    IsDynamic = true;
}

float Rigidbody::InvMass() const {
    if (mass == 0) return 0;
    
    float invMass = 1 / mass;
    
    // If the mass is too big, the float becomes subnormal, so it's clamped to the closest float to 0
    if (std::fpclassify(invMass) == FP_SUBNORMAL)
        return std::numeric_limits<float>::min();
    
    return invMass;
}

void Rigidbody::ApplyForce(const mathpls::vec3& force) {
    acceleration += force * InvMass();
}

void Rigidbody::SetPosition(const mathpls::vec3& pos) {
    lastPostion = pos;
    Position() = pos;
}

void Rigidbody::Translate(const mathpls::vec3& offset) {
    Position() += offset;
}

}
