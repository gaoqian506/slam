#ifndef __WW_SPACE_TOOLBOX_HEADER__
#define __WW_SPACE_TOOLBOX_HEADER__

#include "Vectors.h"
#include "Camera.h"
#include <vector>


namespace ww {



class SpaceToolbox {

public:

static std::vector<Vec3d> unproject(const std::vector<Vec3d>& from);

static std::vector<Vec3d> unproject(const Intrinsic& intrinsic, const std::vector<Vec3d>& from);

};


}

#endif
