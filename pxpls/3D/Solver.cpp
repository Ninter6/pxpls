//
//  Solver.cpp
//  pxpls
//
//  Created by Ninter6 on 2024/4/27.
//

#include "Solver.hpp"

namespace pxpls {

void VerletSolver::Solve(const std::vector<Collision>& collisions, float deltaTime) {
    for (auto& col : collisions) {
        constexpr float response_coef = 1.0f;
        
        auto& obj_1 = *col.A;
        auto& obj_2 = *col.B;
        
        const auto& normal  = col.points.Normal;
        const auto& depth = col.points.Depth;
        
        const float delta  = response_coef * 0.5f * depth;
        const auto col_vec = normal * delta;
        obj_1.Position() += col_vec;
        obj_2.Position() -= col_vec;
    }
}

}
