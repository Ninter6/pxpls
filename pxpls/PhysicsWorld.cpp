//
//  PhysicsWorld.cpp
//  pxpls
//
//  Created by Ninter6 on 2023/10/14.
//

#include "PhysicsWorld.hpp"

namespace pxpls {

void PhysicsWorld::AddObject(PhysicalObj* obj) {
    if (obj) m_Objs.push_back(obj);
}

void PhysicsWorld::RemoveObject(PhysicalObj* obj) {
    if (!obj) return;
    std::erase(m_Objs, obj);
}

void PhysicsWorld::Update(float dt) const {
    for (auto obj : m_Objs) {
        if (!obj->HasRigidBody()) continue;
        auto& rb = *obj->rigidbody();
        
        rb.acceleration += gravity;
        
        rb.velocity += rb.acceleration * dt;
        obj->transform.Position += rb.acceleration * dt;
        
        rb.acceleration = {0};
    }
}

}
