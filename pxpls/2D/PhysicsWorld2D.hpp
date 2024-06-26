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

class PhaseGrid2D {
public:
    virtual ~PhaseGrid2D() = default;
    
    virtual void Update(const CollisionBody2D::Map& bodies) = 0;
    virtual std::vector<CollisionPair2D> GetCollisionPairs() const = 0;
};

class UniformGird2D : public PhaseGrid2D {
public:
    /**
     * \brief Constructs a new grid.
     * \param bound Bound of the whole gird
     * \param cellSize Size (in meter) of a cell.
     */
    UniformGird2D(const Bounds2D& bound, float cellSize);

    /**
     * \brief Updates the layout of the grid.
     * \param bodies Bodies in the physical world.
     */
    virtual void Update(const CollisionBody2D::Map& bodies) override;

    /**
     * \brief Find all the pair of objects that are in the same cell.
     * Doesn't contain any duplicates.
     * \return The pair of objects that will collide.
     */
    virtual std::vector<CollisionPair2D> GetCollisionPairs() const override;

private:
    std::vector<std::vector<std::vector<CollisionBody2D*>>> m_Grid;
    Bounds2D m_Bounds;
    float m_CellSize;

    static bool HasBeenChecked(
        const std::unordered_multimap<CollisionBody2D*, CollisionBody2D*>& checkedCollisions,
        const std::pair<CollisionBody2D*, CollisionBody2D*>& bodyPair);
};

class QuadTree : public PhaseGrid2D {
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
        bool Insert(CollisionBody2D* body);
        
        /**
         * \brief Subdivide the node.
         */
        void Split();
        
        // Data
        Bounds2D bounds;
        std::vector<CollisionBody2D*> bodies;
        
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
    
    virtual void Update(const CollisionBody2D::Map& bodies) override;
    virtual std::vector<CollisionPair2D> GetCollisionPairs() const override;
    
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
class CollisionWorld2D {
public:
    CollisionWorld2D(std::unique_ptr<PhaseGrid2D> phaseGrid);
    
    /**
     * \brief Set the bodies in the world
     * \param bodies Bodies to set.
     */
    void SetCollisionBodies(const CollisionBody2D::Map& bodies);
    
    CollisionBody2D::Map GetCollisionBodies() const;
    
    /**
     * \brief Set the solvers of the world
     * \param solvers solvers to set.
     */
    void SetSolvers(const std::vector<Solver2D*>& solvers);
    
    /**
     * \brief Adds a collision body to the world.
     * \param body Body to add.
     */
    void AddCollisionBody(CollisionBody2D* body);
    
    /**
     * \brief Removes a collision body to the world.
     * \param body Body to remove.
     */
    void RemoveCollisionBody(const CollisionBody2D* body);
    
    /**
     * \brief Adds a solver to the world.
     * \param solver Solver to add.
     */
    void AddSolver(Solver2D* solver);
    
    /**
     * \brief Removes the solver from the world.
     * \param solver Solver to remove.
     */
    void RemoveSolver(Solver2D* solver);
    
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
    void SolveCollisions(const std::vector<Collision2D>& collisions, float deltaTime) const;
    
    /**
     * \brief Calls the callbacks of all the bodies in each collisions.
     * \param collisions Collisions to send callback from.
     * \param deltaTime Time elapsed since the last frame.
     */
    void SendCollisionCallbacks(std::vector<Collision2D>& collisions, float deltaTime) const;
    
    /**
     * \brief Resolves all the collisions that happened in this world.
     * \param deltaTime Time elapsed since the last frame.
     */
    void ResolveCollisions(float deltaTime);
    
private:
    CollisionBody2D::Map m_Bodies;
    std::vector<Solver2D*> m_Solvers;
    std::unique_ptr<PhaseGrid2D> m_Grid;

    std::function<void(Collision2D&, float)> m_OnCollision;
    
    friend class DynamicsWorld2D;
};

/**
 * \brief Represents a world where springs exist.
 */
class SpringWorld2D {
public:
    SpringWorld2D() = default;
    
    /**
     * \brief Adds a spring to the world.
     * \param spring spring to add.
     * \result If the addition is successful.
     */
    bool AddSpring(const Spring2D& spring);
    
    /**
     * \brief Removes a spring from the world.
     * \param link link of the spring (order doesnt matter)
     * \result If the removing is successful.
     */
    bool RemoveSpring(const Link2D& link);
    
    void ResolveSprings() const;
    
private:
    std::unordered_map<Link2D, Spring2D> m_Springs;
    
};

/**
 * \brief Auxiliary struct for calculating gravity
 */
struct GravityFn2D {
    // Rigidbody(read-only), Acceleration
    using Func = std::function<void(const Rigidbody2D*, mathpls::vec2&)>;
    
    GravityFn2D() = default;
    GravityFn2D(mathpls::vec2 gravity);
    GravityFn2D(float x, float y);
    GravityFn2D(Func calcuFunc);
    
    void operator()(const Rigidbody2D* rb, mathpls::vec2& acc) const;
    
    Func func = nullptr;
    mathpls::vec2 g{};
};

/**
 * \brief Evolution mode, used to determine evolutionary algorithms
 */
enum class Evolution {
    Euler,
    Verlet_Postion,
    Verlet_Velocity
};

/**
 * \brief A world with dynamics in it.
 */
class DynamicsWorld2D : public CollisionWorld2D, public SpringWorld2D {
public:
    using CollisionWorld2D::CollisionWorld2D; // inherit constructor
    
    /**
     * \brief Adds a rigidbody in the world.
     * \param rigidbody Rigidbody to add.
     */
    void AddRigidbody(Rigidbody2D* rigidbody);

    /**
     * \brief Removes a rigidbody from the world.
     * \param rigidbody Rigidbody to remove.
     */
    void RemoveRigidbody(Rigidbody2D* rigidbody);
    
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
    
    GravityFn2D Gravity = {{0, -9.81f}};
};

}
