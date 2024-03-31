//
//  PhysicalObj.hpp
//  pxpls
//
//  Created by Ninter6 on 2023/10/14.
//

#pragma once

#include <optional>
#include <cassert>

#include "Transform.hpp"
#include "Collision.hpp"
#include "RigidBody.hpp"

namespace pxpls {

class PhysicalObj {
public:
    PhysicalObj() = default;
    PhysicalObj(const PhysicalObj&) = delete;
    PhysicalObj& operator=(const PhysicalObj&) = delete;
    PhysicalObj(PhysicalObj&&) = default;
    PhysicalObj& operator=(PhysicalObj&&) = default;
    
    Transform transform{};
    
    bool HasCollider() const;
    
    template <class T, class... Args>
    std::enable_if_t<std::is_base_of_v<Collider, T>, T&>
    SetCollider(Args... args) {
        m_Collider.reset(new T{std::forward<Args>(args)...});
        return collider<T>();
    }
    
    template <class T = Collider>
    std::enable_if_t<std::is_base_of_v<Collider, T>, T&>
    collider() {
        assert(m_Collider && "Call collider() but no collider");
        
        auto p = dynamic_cast<T*>(m_Rigidbody.get());
        assert(p && "Call collider() with wrong type!");
        
        return *p;
    }
    
    bool HasRigidBody() const;
    RigidBody& SetRigidbody();
    RigidBody& rigidbody();
    
private:
    
    std::unique_ptr<Collider> m_Collider;
    std::unique_ptr<RigidBody> m_Rigidbody;
};

}
