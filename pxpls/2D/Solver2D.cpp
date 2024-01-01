//
//  Solver2D.cpp
//  pxpls
//
//  Created by Ninter6 on 2023/12/30.
//

#include "Solver2D.hpp"
#include "Dynamic2D.hpp"

namespace pxpls {

void ImpulseSolver::Solve(const std::vector<Collision> &collisions, float deltaTime) {
    for (const auto& [bodyA, bodyB, points] : collisions) {
        // ReSharper disable CppCStyleCast
        Rigidbody* aBody = bodyA->IsDynamic ? (Rigidbody*)bodyA : nullptr;
        Rigidbody* bBody = bodyB->IsDynamic ? (Rigidbody*)bodyB : nullptr;
        // ReSharper restore CppCStyleCast
        
        mathpls::vec2 aVel = aBody ? aBody->Velocity : mathpls::vec2{0};
        mathpls::vec2 bVel = bBody ? bBody->Velocity : mathpls::vec2{0};
        mathpls::vec2 relativeVelocity = bVel - aVel;
        
        // Calculate relative velocity in terms of the normal direction
        float velocityAlongNormal = mathpls::dot(relativeVelocity,  points.Normal);
        
        // Do not resolve if velocities are separating
        if (velocityAlongNormal >= 0) continue;
        
        const float aInvMass = aBody ? aBody->InvMass() : 1.0f;
        const float bInvMass = bBody ? bBody->InvMass() : 1.0f;
        
        // Impulse
        
        const float e = std::min(aBody ? aBody->Restitution : 1.0f, bBody ? bBody->Restitution : 1.0f);
        const float j = -(1.14f + e) * velocityAlongNormal / (aInvMass + bInvMass);
        
        const auto impulse = j * points.Normal;
        
        if (aBody ? aBody->IsKinematic : false) {
            aVel -= impulse * aInvMass;
        }
        
        if (bBody ? bBody->IsKinematic : false) {
            bVel += impulse * bInvMass;
        }
        
        // Friction
        relativeVelocity = bVel - aVel;
        velocityAlongNormal = mathpls::dot(relativeVelocity, points.Normal);
        
        auto tangent = (relativeVelocity - velocityAlongNormal * points.Normal).normalized();
        
        const float fVel = mathpls::dot(relativeVelocity, tangent);
        
        const float aSf = aBody ? aBody->StaticFriction : 0.0f;
        const float bSf = bBody ? bBody->StaticFriction : 0.0f;
        const float aDf = aBody ? aBody->DynamicFriction : 0.0f;
        const float bDf = bBody ? bBody->DynamicFriction : 0.0f;
        float mu = mathpls::vec2(aSf, bSf).length();
        const float f = -fVel / (aInvMass + bInvMass);
        
        mathpls::vec2 friction;
        if (std::abs(f) < j * mu) {
            friction = f * tangent;
        } else {
            mu = mathpls::vec2(aDf, bDf).length();
            friction = -j * tangent * mu;
        }
        
        if (aBody ? aBody->IsKinematic : false) {
            aBody->Velocity = aVel - friction * aInvMass;
        }
        
        if (bBody ? bBody->IsKinematic : false) {
            bBody->Velocity = bVel + friction * bInvMass;
        }
    }
}

void SmoothPositionSolver::Solve(const std::vector<Collision>& collisions, float deltaTime) {
    for (const auto& [bodyA, bodyB, points] : collisions) {
        // ReSharper disable CppCStyleCast
        Rigidbody* aBody = bodyA->IsDynamic ? (Rigidbody*)bodyA : nullptr;
        Rigidbody* bBody = bodyB->IsDynamic ? (Rigidbody*)bodyB : nullptr;
        // ReSharper restore CppCStyleCast

        const float aInvMass = aBody ? aBody->InvMass() : 0.0f;
        const float bInvMass = bBody ? bBody->InvMass() : 0.0f;

        constexpr float slop = 0.01f;
        constexpr float percent = 0.8f;

        const auto correction = points.Normal * percent
            * std::max(points.Depth - slop, 0.0f)
            / (aInvMass + bInvMass);

        if (aBody ? aBody->IsKinematic : false) {
            const auto deltaA = aInvMass * correction;
            aBody->Transform.Position -= deltaA;
        }

        if (bBody ? bBody->IsKinematic : false) {
            const auto deltaB = bInvMass * correction;
            bBody->Transform.Position += deltaB;
        }
    }
}

}
