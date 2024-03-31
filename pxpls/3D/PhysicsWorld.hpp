//
//  PhysicsWorld.hpp
//  pxpls
//
//  Created by Ninter6 on 2023/10/14.
//

#pragma once

#include "PhysicalObj.hpp"
#include "Geometry.hpp"

#include <vector>

namespace pxpls {

using CollisionPair = std::pair<CollisionBody::id_t, CollisionBody::id_t>;

class PhaseGrid {
public:
    virtual ~PhaseGrid() = default;
    
    virtual void Update(const CollisionBody::Map& bodies) = 0;
    virtual std::vector<CollisionPair> GetCollisionPairs() const = 0;
};

class UniformGrid : public PhaseGrid {
public:
    UniformGrid() = default;
    UniformGrid(const Bounds& bounds, mathpls::uivec3 size);
    
    virtual void Update(const CollisionBody::Map& bodies) override;
    virtual std::vector<CollisionPair> GetCollisionPairs() const override;
    
private:
    std::vector<std::vector<std::vector<std::vector<CollisionBody*>>>> m_Grid;
    Bounds m_Bounds;
    
    void clear();
    
};

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
