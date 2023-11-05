//
//  PhysicalObj.cpp
//  pxpls
//
//  Created by Ninter6 on 2023/10/14.
//

#include "PhysicalObj.hpp"

#include <cassert>

namespace pxpls {

bool PhysicalObj::HasCollider() {
    return m_Collider != nullptr;
}

std::unique_ptr<Collider>& PhysicalObj::collider() {
    assert(m_Collider && "Call collider() but no collider");
    return m_Collider;
}

bool PhysicalObj::HasRigidBody() {
    return m_Rigidbody != nullptr;
}

std::unique_ptr<RigidBody>& PhysicalObj::rigidbody() {
    assert(m_Rigidbody && "Call rigidbody() but no rigidbody");
    return m_Rigidbody;
}

}
