//
//  PhysicalObj.cpp
//  pxpls
//
//  Created by Ninter6 on 2023/10/14.
//

#include "PhysicalObj.hpp"

#include <cassert>

namespace pxpls {

//PhysicalObj::PhysicalObj(const PhysicalObj& o) {
//    
//}
//
//PhysicalObj& PhysicalObj::operator=(const PhysicalObj& o) {
//    
//    
//    return *this;
//}

bool PhysicalObj::HasCollider() const {
    return m_Collider != nullptr;
}

bool PhysicalObj::HasRigidBody() const {
    return m_Rigidbody != nullptr;
}

RigidBody& PhysicalObj::SetRigidbody() {
    m_Rigidbody.reset(new RigidBody);
    return rigidbody();
}

RigidBody& PhysicalObj::rigidbody() {
    assert(m_Rigidbody && "Call rigidbody() but no rigidbody");
    return *m_Rigidbody;
}

}
