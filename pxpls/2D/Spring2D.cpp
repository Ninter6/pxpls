//
//  Spring2D.cpp
//  pxpls
//
//  Created by Ninter6 on 2024/2/12.
//

#include "Spring2D.hpp"
#include "Dynamic2D.hpp"

#include <cfloat>

namespace pxpls {

void Spring::CalcuForce() const {
    auto A2B = B()->Position() - A()->Position();
    auto dis = A2B.length();
    A2B.normalize();
    
    auto f = K * A2B * (dis - RestLength);
    if (Dashpot != 0) {
        f += -Dashpot * mathpls::project(A()->Velocity - B()->Velocity, -A2B);
    }
    A()->Force += f;
    B()->Force -= f;
}

}
