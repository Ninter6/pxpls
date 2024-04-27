//
//  PhysicsWorld2D.cpp
//  pxpls
//
//  Created by Ninter6 on 2023/12/30.
//

#include "PhysicsWorld2D.hpp"

#include <unordered_set>
#include <cassert>

template<>
struct std::hash<pxpls::CollisionPair> {
    size_t operator()(const pxpls::CollisionPair& p) const {
        auto h1 = std::hash<uint64_t>{}(p.first), h2 = std::hash<uint64_t>{}(p.second);
        return h1 ^ ((h1 << 32) & (h2 >> 32));
    }
};

namespace pxpls {

UniformGird::UniformGird(const Bounds2D& bounds, float cellSize)
: m_Bounds(bounds), m_CellSize(cellSize) {
    std::vector<std::vector<CollisionBody*>> height(
        static_cast<size_t>((bounds.max.y - bounds.min.y) / cellSize));
    m_Grid.resize(static_cast<size_t>((bounds.max.x - bounds.min.x) / cellSize), height);
}

void UniformGird::Update(const CollisionBody::Map& bodies) {
    // clear
    for (auto& gridCol : m_Grid)
        for (auto& gridCell : gridCol)
            gridCell.clear();
    
    // process bodies
    for (const auto& [_, body] : bodies) {
        const auto& transform = body->Transform;
        const auto& collider = body->Collider;

        const auto bounds = collider->GetBounds(&transform);

        // If body is outside the grid extents, then ignore it
        if (!m_Bounds.IsOverlapping(bounds))
            continue;
        
        auto offsetmin = (bounds.min - m_Bounds.min) / m_CellSize;
        auto offsetmax = (bounds.max - m_Bounds.min) / m_CellSize;
        
        mathpls::vec<size_t, 2> imin = {
            std::clamp<size_t>(size_t(offsetmin.x), 0, m_Grid.size() - 1),
            std::clamp<size_t>(size_t(offsetmin.y), 0, m_Grid.size() - 1)
        };
        mathpls::vec<size_t, 2> imax = {
            std::clamp<size_t>(size_t(offsetmax.x), 0, m_Grid.size() - 1),
            std::clamp<size_t>(size_t(offsetmax.y), 0, m_Grid.size() - 1)
        };
        
        for (size_t i = imin.x; i <= imax.x; ++i)
            for (size_t j = imin.y; j <= imax.y; ++j)
                m_Grid[i][j].push_back(body);
    }
}

std::vector<CollisionPair> UniformGird::GetCollisionPairs() const {
    std::unordered_set<CollisionPair> pairs;
    
    for (auto& gridCol : m_Grid)
        for (auto& gridCell : gridCol)
            if (gridCell.size() > 1)
                for (int i = 0; i < gridCell.size() - 1; ++i)
                    for (int j = i + 1; j < gridCell.size(); ++j)
                        pairs.emplace(gridCell[i]->id, gridCell[j]->id);
    
    return {pairs.begin(), pairs.end()};
}

QuadTree::Node::Node(const Bounds2D& bounds, uint32_t currDepth)
: bounds(bounds), currDepth(currDepth) {}

bool QuadTree::Node::IsLeaf() const {
    return children.empty(); // each node must have 0 or 4 children
}

bool QuadTree::Node::Insert(CollisionBody* body) {
    if (IsLeaf() && bounds.IsOverlapping(body->GetBounds())) {
        bodies.push_back(body);
        return true;
    }
    
    return false;
}

void QuadTree::Node::Split() {
    auto center = bounds.center();
    const auto& [min, max] = bounds;
    
    children.reserve(4);
    children.emplace_back(Bounds2D{min, center}, currDepth + 1); // left bottom
    children.emplace_back(Bounds2D{{min.x, center.y}, {center.x, max.y}}, currDepth + 1); // left top
    children.emplace_back(Bounds2D{{center.x, min.y}, {max.x, center.y}}, currDepth + 1); // right bottom
    children.emplace_back(Bounds2D{center, max}, currDepth + 1);  // right top
    
    for (auto i : bodies)
        for (auto& c : children)
            c.Insert(i);
    
    // clear and delete
    std::vector<CollisionBody*> empty{};
    bodies.swap(empty);
}

QuadTree::Iterator::Iterator(Node* root) {
    if (root) {
        node_que.push(root);
        MoveToNextLeaf();
    }
}

void QuadTree::Iterator::MoveToNextLeaf() {
    if (!node_que.empty())
        while (!node_que.front()->IsLeaf()) {
            for (auto& i : node_que.front()->children) {
                node_que.push(&i);
            }
            node_que.pop();
        }
}

QuadTree::Node& QuadTree::Iterator::operator*() const {
    assert(!node_que.empty() && "illegal dereferencing");
    return *node_que.front();
}

QuadTree::Iterator& QuadTree::Iterator::operator++() {
    if (node_que.front()->IsLeaf())
        node_que.pop();
    MoveToNextLeaf();
    return *this;
}

QuadTree::Iterator QuadTree::Iterator::operator++(int) {
    auto old = *this;
    ++*this;
    return old;
}

bool operator==(const QuadTree::Iterator& a, const QuadTree::Iterator& b) {
    return (a.node_que.empty() && b.node_que.empty()) ||
    (!a.node_que.empty() && !b.node_que.empty() && a.node_que.front() == b.node_que.front());
}

bool operator!=(const QuadTree::Iterator& a, const QuadTree::Iterator& b) {
    return !(a == b);
}

QuadTree::QuadTree(const Bounds2D& rootBound, uint32_t maxDepth, uint32_t maxObjsPreNode)
: m_Root(new Node(rootBound, 0)), maxDepth(maxDepth), maxObjsPreNode(maxObjsPreNode) {}

QuadTree::Iterator QuadTree::begin() {
    return Iterator(m_Root.get());
}

QuadTree::Iterator QuadTree::end() {
    return {};
}

const QuadTree::Iterator QuadTree::begin() const {
    return Iterator(m_Root.get());
}

const QuadTree::Iterator QuadTree::end() const {
    return {};
}

void QuadTree::Update(const CollisionBody::Map& bodies) {
    m_Root.reset(new Node(m_Root->bounds, 0));
    for (const auto& [_, body] : bodies)
        m_Root->Insert(body);
    for (auto& node : *this) {
        if (node.currDepth >= maxDepth) break;
        if (node.bodies.size() > maxObjsPreNode)
            node.Split();
    }
}

std::vector<CollisionPair> QuadTree::GetCollisionPairs() const {
    std::unordered_set<CollisionPair> pairs;
    for (auto& node : *this)
        if (node.bodies.size() > 1)
            for (int i = 0; i < node.bodies.size() - 1; ++i)
                for (int j = i + 1; j < node.bodies.size(); ++j) {
                    //                    auto a = node.bodies[i], b = node.bodies[j];
                    //                    if (a < b) pairs.emplace(a->id, b->id);
                    //                    if (a > b) pairs.emplace(b->id, a->id);
                    pairs.emplace(node.bodies[i]->id, node.bodies[j]->id);
                }
    
    return {pairs.begin(), pairs.end()};
}

CollisionWorld::CollisionWorld(std::unique_ptr<PhaseGrid> phaseGrid)
: m_Grid(std::move(phaseGrid)) {}

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
        
        if (!a->Collider || !b->Collider) continue;
        
        const auto points = a->Collider->TestCollision(&a->Transform,
                                                       b->Collider,
                                                       &b->Transform);
        
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

bool SpringWorld::AddSpring(const Spring& spring) {
    return m_Springs.emplace(spring.Link, spring).second;
}

bool SpringWorld::RemoveSpring(const Link &link) {
    return m_Springs.erase(link);
}

void SpringWorld::ResolveSprings() const {
    for (auto& [_, i] : m_Springs)
        i.CalcuForce();
}

GravityFn::GravityFn(mathpls::vec2 gravity) : g(gravity) {}

GravityFn::GravityFn(float x, float y) : g(x, y) {}

GravityFn::GravityFn(Func calcuFunc) : func(calcuFunc) {}

void GravityFn::operator()(const Rigidbody* rb, mathpls::vec2& acc) const {
    if (func) func(rb, acc);
    else acc += g;
}

void DynamicsWorld::AddRigidbody(Rigidbody *rigidbody) {
    AddCollisionBody(rigidbody);
}

void DynamicsWorld::RemoveRigidbody(Rigidbody* rigidbody) {
    RemoveCollisionBody(rigidbody);
}

void DynamicsWorld::MoveBodies(const float deltaTime, Evolution mode) const {
    for (const auto& [_, body] : m_Bodies) {
        if (!body->IsDynamic) continue;
        const auto rigidbody = (Rigidbody*)body;
        
        mathpls::vec2 a = rigidbody->Force * rigidbody->InvMass();
        if (rigidbody->TakesGravity)
            Gravity(rigidbody, a); // Apply Gravity
        
        switch (mode) {
                // semi-implicit Euler method
            case Evolution::Euler: {
                auto dv = a * deltaTime;
                rigidbody->Velocity += dv;
                rigidbody->Position() += (rigidbody->Velocity + dv) * deltaTime;
                break;
            }
                // Velocity Verlet
            case Evolution::Verlet_Postion:
                rigidbody->Position() += rigidbody->Velocity * deltaTime + a * .5f * deltaTime * deltaTime;
            case Evolution::Verlet_Velocity:
                rigidbody->Velocity += a * deltaTime * .5f;
                break;
        }
        
        rigidbody->Force = {0};
    }
}

void DynamicsWorld::Step(float deltaTime) {
    // COLLISION
    // no force generated
    ResolveCollisions(deltaTime);
    
    // SPRING
    // p(t+dt) = p(t) + v(t)dt + a(t) * dt^2 / 2
    // v(t+dt) = v(t) + (a(t+dt) + a(t)) * dt / 2
    // divided into two calculations
    ResolveSprings();
    MoveBodies(deltaTime, Evolution::Verlet_Postion);
    ResolveSprings();
    MoveBodies(deltaTime, Evolution::Verlet_Velocity);
}

}
