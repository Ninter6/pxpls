//
//  PhysicsWorld.hpp
//  pxpls
//
//  Created by Ninter6 on 2023/10/14.
//

#pragma once

#include "Collision.hpp"
#include "Solver.hpp"
#include "Dynamic.hpp"
#include "Geometry.hpp"

#include <vector>
#include <memory>

namespace pxpls {

class PhaseGrid {
public:
    virtual ~PhaseGrid() = default;
    
    virtual void Update(const CollisionBody::Map& bodies) = 0;
    virtual std::vector<CollisionPair> GetCollisionPairs() const = 0;
    
    virtual void Clear() = 0;
};

class UniformGrid : public PhaseGrid {
public:
    UniformGrid() = default;
    UniformGrid(const Bounds& bounds, mathpls::uivec3 size);
    
    virtual void Update(const CollisionBody::Map& bodies) override;
    virtual std::vector<CollisionPair> GetCollisionPairs() const override;
    
    virtual void Clear() override;
    
private:
    std::vector<std::vector<std::vector<std::vector<CollisionBody*>>>> m_Grid;
    Bounds m_Bounds;
    
};

/**
 * \brief Represents a world where collisions can happen.
 * It does not have dynamics, for this you will need the DynamicsWorld.
 * \see DynamicsWorld
 */
class CollisionWorld {
public:
    CollisionWorld(std::unique_ptr<PhaseGrid> phaseGrid);
    
    PhaseGrid* GetGrid() const;
    
    /**
     * \brief Set the bodies in the world
     * \param bodies Bodies to set.
     */
    void SetCollisionBodies(const CollisionBody::Map& bodies);
    
    CollisionBody::Map GetCollisionBodies() const;
    
    /**
     * \brief Set the solvers of the world
     * \param solvers solvers to set.
     */
    void SetSolvers(const std::vector<Solver*>& solvers);
    
    /**
     * \brief Adds a collision body to the world.
     * \param body Body to add.
     */
    void AddCollisionBody(CollisionBody* body);
    
    /**
     * \brief Removes a collision body to the world.
     * \param body Body to remove.
     */
    void RemoveCollisionBody(const CollisionBody* body);
    
    /**
     * \brief Adds a solver to the world.
     * \param solver Solver to add.
     */
    void AddSolver(Solver* solver);
    
    /**
     * \brief Removes the solver from the world.
     * \param solver Solver to remove.
     */
    void RemoveSolver(Solver* solver);
    
    /**
     * \brief Sets the collision callback of this world.
     * \param callback Callback to set.
     */
    void SetCollisionCallback(const CollisionCallback& callback);
    
    /**
     * \brief Solves the collisions with the provided solvers.
     * \param collisions Collisions to solve.
     * \param deltaTime Time elapsed since the last frame.
     */
    void SolveCollisions(const std::vector<Collision>& collisions, float deltaTime) const;
    
    /**
     * \brief Calls the callbacks of all the bodies in each collisions.
     * \param collisions Collisions to send callback from.
     * \param deltaTime Time elapsed since the last frame.
     */
    void SendCollisionCallbacks(std::vector<Collision>& collisions, float deltaTime) const;
    
    /**
     * \brief Resolves all the collisions that happened in this world.
     * \param deltaTime Time elapsed since the last frame.
     */
    void ResolveCollisions(float deltaTime);
    
private:
    CollisionBody::Map m_Bodies;
    std::vector<Solver*> m_Solvers;
    std::unique_ptr<PhaseGrid> m_Grid;

    std::function<void(Collision&, float)> m_OnCollision;
    
    friend class DynamicsWorld;
};

/**
 * \brief Auxiliary struct for calculating gravity
 */
struct GravityFn {
    // Rigidbody(read-only), Acceleration
    using Func = std::function<void(const Rigidbody*, mathpls::vec3&)>;
    
    GravityFn() = default;
    GravityFn(mathpls::vec3 gravity);
    GravityFn(float x, float y, float z);
    GravityFn(Func calcuFunc);
    
    void operator()(const Rigidbody* rb, mathpls::vec3& acc) const;
    
    Func func = nullptr;
    mathpls::vec3 g{};
};

/**
 * \brief A world with dynamics in it.
 */
class DynamicsWorld : public CollisionWorld {
public:
    using CollisionWorld::CollisionWorld; // inherit constructor
    
    /**
     * \brief Adds a rigidbody in the world.
     * \param rigidbody Rigidbody to add.
     */
    void AddRigidbody(Rigidbody* rigidbody);
    
    /**
     * \brief Removes a rigidbody from the world.
     * \param rigidbody Rigidbody to remove.
     */
    void RemoveRigidbody(Rigidbody* rigidbody);

    /**
     * \brief Moves all the rigidbodies.
     * \param deltaTime Time elapsed since the last frame.
     */
    void MoveBodies(float deltaTime) const;

    /**
     * \brief Steps the world.
     * \param deltaTime Time elapsed since the last frame.
     */
    void Step(float deltaTime);
    
    GravityFn Gravity = {0, -9.81f, 0};
};

}
