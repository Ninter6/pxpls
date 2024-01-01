//
//  Transform2D.h
//  pxpls
//
//  Created by Ninter6 on 2023/12/29.
//

#pragma once

#include "mathpls.h"

namespace pxpls {

struct Transform2D {
    mathpls::vec2 Position{};
    mathpls::vec2 Scale{1.f};
    float Rotation{};
};

}
