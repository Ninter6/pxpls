//
//  PhysicsWorld.cpp
//  pxpls
//
//  Created by Ninter6 on 2023/10/14.
//

#include "PhysicsWorld.hpp"

#include <algorithm>

namespace pxpls {

void PhysicsWorld::AddObject(PhysicalObj* obj) {
    if (obj) m_Objs.push_back(obj);
}

void PhysicsWorld::RemoveObject(PhysicalObj* obj) {
    if (!obj) return;
    auto it = std::find(m_Objs.begin(), m_Objs.end(), obj);
    if (it != m_Objs.end()) m_Objs.erase(it);
}

void PhysicsWorld::Update(float dt) const {
    for (auto obj : m_Objs) {
        if (!obj->HasRigidBody()) continue;
        auto& rb = obj->rigidbody();
        
        rb.acceleration.y -= gravity;
        
        rb.velocity += rb.acceleration * dt;
        obj->transform.Position += rb.velocity * dt;
        
        rb.acceleration = {0};
    }
}

}
