//
//  RigidBody.hpp
//  pxpls
//
//  Created by Ninter6 on 2023/8/5.
//

#pragma once

#include <iostream>
#include "Collider.hpp"

namespace pxpls {

struct RigidBody {
    float mass = 1.f;
    float gravity = 1.f;
    mathpls::vec3 velocity;
    mathpls::vec3 acceleration;
};

}
