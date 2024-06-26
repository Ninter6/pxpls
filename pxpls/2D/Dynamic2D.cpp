//
//  Dynamic2D.cpp
//  pxpls
//
//  Created by Ninter6 on 2023/12/30.
//

#include "Dynamic2D.hpp"

#include <cmath>
#include <limits>

namespace pxpls {

Rigidbody2D::Rigidbody2D() : Mass(1), TakesGravity(true) {
    IsDynamic = true;
}

float Rigidbody2D::InvMass() const {
    if (Mass == 0) return 0;
    
    float invMass = 1 / Mass;
    
    // If the mass is too big, the float becomes subnormal, so it's clamped to the closest float to 0
    if (std::fpclassify(invMass) == FP_SUBNORMAL)
        return std::numeric_limits<float>::min();
    
    return invMass;
}

mathpls::vec2& Rigidbody2D::ApplyForce(mathpls::vec2 addedForce) {
    return Force += addedForce;
}

mathpls::vec2 Rigidbody2D::SetPostion(const mathpls::vec2& pos) {
    LastPosition = Position();
    Position() = pos;
    return LastPosition;
}

}
