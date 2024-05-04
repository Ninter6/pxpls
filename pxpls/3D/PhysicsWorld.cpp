//
//  PhysicsWorld.cpp
//  pxpls
//
//  Created by Ninter6 on 2023/10/14.
//

#include "PhysicsWorld.hpp"

#include <algorithm>
#include <unordered_set>

namespace pxpls {

UniformGrid::UniformGrid(const Bounds& bounds, mathpls::uivec3 size)
: m_Bounds(bounds) {
    m_Grid.resize(size.x);
    for (auto& i : m_Grid) {
        i.resize(size.y);
        for (auto& j : i)
            j.resize(size.z);
    }
}

void UniformGrid::Update(const CollisionBody::Map& bodies) {
    Clear();
    
    for (auto& [id, body] : bodies) {
        if (!body->collider) continue;

        auto bnd = body->GetBounds();
        
        if (!m_Bounds.overlapping(bnd)) continue;
        
        mathpls::vec3 len = m_Bounds.max - m_Bounds.min;
        const auto& min = m_Bounds.min;
        
        mathpls::vec3 sz;
        sz.x = m_Grid.size();
        sz.y = m_Grid[0].size();
        sz.z = m_Grid[0][0].size();
        
        mathpls::vec3 b{
            std::clamp<float>(sz.x * (bnd.min.x - min.x) / len.x, 0, sz.x - 1),
            std::clamp<float>(sz.y * (bnd.min.y - min.y) / len.y, 0, sz.y - 1),
            std::clamp<float>(sz.z * (bnd.min.z - min.z) / len.z, 0, sz.z - 1)
        };
        
        mathpls::vec3 e{
            std::clamp<float>(sz.x * (bnd.max.x - min.x) / len.x, 0, sz.x - 1),
            std::clamp<float>(sz.y * (bnd.max.y - min.y) / len.y, 0, sz.y - 1),
            std::clamp<float>(sz.z * (bnd.max.z - min.z) / len.z, 0, sz.z - 1)
        };
        
        for (int i = b.x; i <= e.x; i++)
            for (int j = b.y; j <= e.y; j++)
                for (int k = b.z; k <= e.z; k++)
                    m_Grid[i][j][k].push_back(body);
    }
}

std::vector<CollisionPair> UniformGrid::GetCollisionPairs() const {
    std::unordered_set<CollisionPair> s;
    
    for (auto& i : m_Grid)
        for (auto& j : i)
            for (auto& k : j) {
                if (k.size() < 2) continue;
                for (int m = 0; m < k.size() - 1; m++)
                    for (int n = m + 1; n < k.size(); n++) {
                        if (k[m] < k[n]) s.emplace(k[m]->id, k[n]->id);
                        if (k[n] < k[m]) s.emplace(k[n]->id, k[m]->id);
                    }
            }
    
    return {s.begin(), s.end()};
}

void UniformGrid::Clear() {
    for (auto& i : m_Grid)
        for (auto& j : i)
            for (auto& k : j)
                k.clear();
}


CollisionWorld::CollisionWorld(std::unique_ptr<PhaseGrid> phaseGrid)
: m_Grid(std::move(phaseGrid)) {}

PhaseGrid* CollisionWorld::GetGrid() const {
    return m_Grid.get();
}

void CollisionWorld::SetCollisionBodies(const CollisionBody::Map& bodies) {
    m_Bodies = bodies;
}

CollisionBody::Map CollisionWorld::GetCollisionBodies() const {
    return m_Bodies;
}

void CollisionWorld::SetSolvers(const std::vector<Solver*>& solvers) {
    m_Solvers = solvers;
}

void CollisionWorld::AddCollisionBody(CollisionBody* body) {
    if (body) m_Bodies.insert({body->id, body});
}

void CollisionWorld::RemoveCollisionBody(const CollisionBody* body) {
    if (body) m_Bodies.erase(body->id);
}

void CollisionWorld::AddSolver(Solver* solver) {
    if (solver) m_Solvers.push_back(solver);
}

void CollisionWorld::RemoveSolver(Solver* solver) {
    auto it = std::find(m_Solvers.begin(), m_Solvers.end(), solver);
    if (it != m_Solvers.end()) {
        m_Solvers.erase(it);
    }
}

void CollisionWorld::SetCollisionCallback(const CollisionCallback& callback) {
    if (callback) m_OnCollision = callback;
}

void CollisionWorld::SolveCollisions(const std::vector<Collision>& collisions,
                                     float deltaTime) const {
    for (auto& solver : m_Solvers) {
        solver->Solve(collisions, deltaTime);
    }
}

void CollisionWorld::SendCollisionCallbacks(std::vector<Collision>& collisions,
                                            float deltaTime) const {
    for (auto& collision : collisions) {
        if (m_OnCollision)
            m_OnCollision(collision, deltaTime);
        
        collision.A->OnCollision(collision, deltaTime);
        collision.B->OnCollision(collision, deltaTime);
    }
}

void CollisionWorld::ResolveCollisions(float deltaTime) {
    // Vector for the collisions that have been detected
    std::vector<Collision> collisions;
    
    // Vector for the collisions that have been caused by trigger colliders
    std::vector<Collision> triggers;
    
    // Update the grid and find the object that can collide together
    m_Grid->Update(m_Bodies);
    const auto collisionPairs = m_Grid->GetCollisionPairs();
    
    for (auto& [firstId, secondId] : collisionPairs) {
        CollisionBody* a = m_Bodies[firstId];
        CollisionBody* b = m_Bodies[secondId];
        
        if (!a->collider || !b->collider) continue;
        
        const auto points = a->collider->TestCollision(a->transform,
                                                       *b->collider,
                                                       b->transform);
        
        if (!points.HasCollision) continue;
        
        Collision g{a, b, points};
        if (a->IsTrigger || b->IsTrigger) {
            triggers.push_back(g);
        } else {
            collisions.push_back(g);
        }
    }
    
    SolveCollisions(collisions, deltaTime);
    
    SendCollisionCallbacks(collisions, deltaTime);
    SendCollisionCallbacks(triggers, deltaTime);
}

GravityFn::GravityFn(mathpls::vec3 gravity) : g(gravity) {}

GravityFn::GravityFn(float x, float y, float z) : g(x, y, z) {}

GravityFn::GravityFn(Func calcuFunc) : func(calcuFunc) {}

void GravityFn::operator()(const Rigidbody* rb, mathpls::vec3& acc) const {
    if (func)
        func(rb, acc);
    else
        acc += g;
}

void DynamicsWorld::AddRigidbody(Rigidbody *rigidbody) {
    AddCollisionBody(rigidbody);
}

void DynamicsWorld::RemoveRigidbody(Rigidbody *rigidbody) {
    RemoveCollisionBody(rigidbody);
}

void DynamicsWorld::MoveBodies(float deltaTime) const {
    for (const auto& [_, body] : m_Bodies) {
        if (!body->IsDynamic) continue;
        const auto rigidbody = (Rigidbody*)body;
        
        auto& a = rigidbody->acceleration;
        if (rigidbody->takeGravity)
            Gravity(rigidbody, a); // Apply Gravity
        
        auto pos = rigidbody->transform.Position;
        
        rigidbody->Translate(pos - rigidbody->lastPostion + a*deltaTime*deltaTime);
        
        rigidbody->lastPostion = pos;
        
        a = {0};
    }
}

void DynamicsWorld::Step(float deltaTime) {
    ResolveCollisions(deltaTime);
    MoveBodies(deltaTime);
}

}
