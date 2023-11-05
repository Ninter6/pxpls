//
//  PhysicalObj.hpp
//  pxpls
//
//  Created by Ninter6 on 2023/10/14.
//

#pragma once

#include <optional>

#include "Transform.hpp"
#include "Collider.hpp"
#include "RigidBody.hpp"

namespace pxpls {

class PhysicalObj {
public:
    PhysicalObj() = default;
    
    TransForm transform{};
    
    bool HasCollider();
    std::unique_ptr<Collider>& collider();
    bool HasRigidBody();
    std::unique_ptr<RigidBody>& rigidbody();
    
private:
    
    std::unique_ptr<Collider> m_Collider;
    std::unique_ptr<RigidBody> m_Rigidbody;
};

}
