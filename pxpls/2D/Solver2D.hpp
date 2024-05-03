//
//  Solver2D.hpp
//  pxpls
//
//  Created by Ninter6 on 2023/12/30.
//

#pragma once

#include "Collision2D.hpp"

namespace pxpls {

class Solver2D {
public:
    virtual ~Solver2D() = default;
    
    /**
     * \brief Solves the provided collisions.
     * \param collisions Collisions to solve.
     * \param deltaTime Time elapsed since the last frame.
     */
    virtual void Solve(const std::vector<Collision2D>& collisions, float deltaTime) = 0;
};

class ImpulseSolver : public Solver2D {
public:
    ImpulseSolver() = default;
    
    virtual void Solve(const std::vector<Collision2D>& collisions, float deltaTime) override;
};

/**
* \brief A solver to smooth out collision with collider that are in a tower placement.
*/
class SmoothPositionSolver : public Solver2D {
public:
    virtual void Solve(const std::vector<Collision2D>& collisions, float deltaTime) override;
};

}
