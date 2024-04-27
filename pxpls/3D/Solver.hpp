//
//  Solver.hpp
//  pxpls
//
//  Created by Ninter6 on 2024/4/27.
//

#pragma once

#include "Collision.hpp"

namespace pxpls {

struct Solver {
    virtual ~Solver() = default;
    
    /**
     * \brief Solves the provided collisions.
     * \param collisions Collisions to solve.
     * \param deltaTime Time elapsed since the last frame.
     */
    virtual void Solve(const std::vector<Collision>& collisions, float deltaTime) = 0;
};

struct VerletSolver : Solver {
    virtual void Solve(const std::vector<Collision>& collisions, float deltaTime) override;
};

}
