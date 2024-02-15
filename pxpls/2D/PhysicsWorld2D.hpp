//
//  PhysicsWorld2D.hpp
//  pxpls
//
//  Created by Ninter6 on 2023/12/30.
//

#pragma once

#include "Collision2D.hpp"
#include "Dynamic2D.hpp"
#include "Solver2D.hpp"
#include "Spring2D.hpp"

#include <memory>
#include <queue>

namespace pxpls {

using CollisionPair = std::pair<CollisionBody::id_t, CollisionBody::id_t>;

class PhaseGrid {
public:
    virtual ~PhaseGrid() = default;
    
    virtual void Update(const CollisionBody::Map& bodies) = 0;
    virtual std::vector<CollisionPair> GetCollisionPairs() const = 0;
};

class UniformGird : public PhaseGrid {
public:
    /**
     * \brief Constructs a new grid.
     * \param bound Bound of the whole gird
     * \param cellSize Size (in meter) of a cell.
     */
    UniformGird(const Bounds2D& bound, float cellSize);

    /**
     * \brief Updates the layout of the grid.
     * \param bodies Bodies in the physical world.
     */
    virtual void Update(const CollisionBody::Map& bodies) override;

    /**
     * \brief Find all the pair of objects that are in the same cell.
     * Doesn't contain any duplicates.
     * \return The pair of objects that will collide.
     */
    virtual std::vector<CollisionPair> GetCollisionPairs() const override;

private:
    std::vector<std::vector<std::vector<CollisionBody*>>> m_Grid;
    Bounds2D m_Bounds;
    float m_CellSize;

    static bool HasBeenChecked(
        const std::unordered_multimap<CollisionBody*, CollisionBody*>& checkedCollisions,
        const std::pair<CollisionBody*, CollisionBody*>& bodyPair);
};

class QuadTree : public PhaseGrid {
public:
    struct Node {
        Node(const Bounds2D& bounds, uint32_t currDepth);
        
        /**
         * \brief Determine whether a node is a leaf node.
         */
        bool IsLeaf() const;
        
        /**
         * \brief Insert a body into the node.
         * \note Only leaves can be insert into.
         * \result Boolean indicating whether insertion was successful.
         */
        bool Insert(CollisionBody* body);
        
        /**
         * \brief Subdivide the node.
         */
        void Split();
        
        // Data
        Bounds2D bounds;
        std::vector<CollisionBody*> bodies;
        
        uint32_t currDepth;
        
        // Children
        std::vector<Node> children;
    };
    
    struct Iterator {
        Iterator(Node* root = nullptr);
        
        Node& operator*() const;
        Iterator& operator++();
        friend bool operator==(const Iterator& a, const Iterator& b);
        friend bool operator!=(const Iterator& a, const Iterator& b);
        
        /**
         * \note this is deprecated
         */
        Iterator operator++(int);
        
        void MoveToNextLeaf();
        
        std::queue<Node*> node_que;
    }; // forward iterator
    
    QuadTree() = default;
    QuadTree(const Bounds2D& rootBound, uint32_t maxDepth = 5, uint32_t maxObjPreNode = 5);
    
    virtual void Update(const CollisionBody::Map& bodies) override;
    virtual std::vector<CollisionPair> GetCollisionPairs() const override;
    
    Iterator begin();
    Iterator end();
    const Iterator begin() const;
    const Iterator end() const;
    
private:
    uint32_t maxDepth, maxObjsPreNode;
    
    std::unique_ptr<Node> m_Root;
    
};

/**
 * \brief Represents a world where collisions can happen.
 * It does not have dynamics, for this you will need the DynamicsWorld.
 * \see DynamicsWorld
 */
class CollisionWorld {
public:
    CollisionWorld(std::unique_ptr<PhaseGrid> phaseGrid);
    
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
 * \brief Represents a world where springs exist.
 */
class SpringWorld {
public:
    SpringWorld() = default;
    
    /**
     * \brief Adds a spring to the world.
     * \param spring spring to add.
     * \result If the addition is successful.
     */
    bool AddSpring(const Spring& spring);
    
    /**
     * \brief Removes a spring from the world.
     * \param link link of the spring (order doesnt matter)
     * \result If the removing is successful.
     */
    bool RemoveSpring(const Link& link);
    
    void ResolveSprings(float deltaTime) const;
    
private:
    std::unordered_map<Link, Spring> m_Springs;
    
};

enum class Evolution {
    Euler,
    Verlet_Postion,
    Verlet_Velocity
};

/**
* \brief A world with dynamics in it.
*/
class DynamicsWorld : public CollisionWorld, public SpringWorld {
public:
    using CollisionWorld::CollisionWorld; // inherit constructor
    
    /**
     * \brief Adds a rigidbody in the world.
     * \param rigidbody Rigidbody to add.
     */
    void AddRigidbody(Rigidbody* rigidbody);

    /**
     * \brief Moves all the rigidbodies.
     * \param deltaTime Time elapsed since the last frame.
     */
    void MoveBodies(float deltaTime, Evolution mode) const;

    /**
     * \brief Steps the world.
     * \param deltaTime Time elapsed since the last frame.
     */
    void Step(float deltaTime);
    
    void substepCollision(float deltaTime);
    void substepString(float deltaTime);

    mathpls::vec2 Gravity = {0, -9.81f};
};



}
