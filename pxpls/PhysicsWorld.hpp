//
//  PhysicsWorld.hpp
//  pxpls
//
//  Created by Ninter6 on 2023/10/14.
//

#pragma once

#include <vector>
#include "PhysicalObj.hpp"

namespace pxpls {

class PhysicsWorld {
public:
    
    void AddObject(PhysicalObj* obj);
    void RemoveObject(PhysicalObj* obj);
    
    void Update(float dt) const;
    
private:
    
    float gravity = 9.8f;
    
    std::vector<PhysicalObj*> m_Objs;
    
};

}
