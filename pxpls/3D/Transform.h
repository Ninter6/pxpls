//
//  Transform.hpp
//  pxpls
//
//  Created by Ninter6 on 2023/10/14.
//

#pragma once

#include "mathpls.h"

namespace pxpls {

struct Transform {
    mathpls::vec3 Position;
    mathpls::vec3 Scale= 1;
    mathpls::quat Rotation;
};

}
