
#include "SpaceToolbox.h"
#include <assert.h>

namespace ww {


std::vector<Vec3d> SpaceToolbox::unproject(const std::vector<Vec3d>& from) {

	assert(0);
	return std::vector<Vec3d>();
	
}

std::vector<Vec3d> SpaceToolbox::unproject(const Intrinsic& intrinsic, const std::vector<Vec3d>& from) {

	std::vector<Vec3d> to;
	double invk[3] = { 1/intrinsic.f, intrinsic.cx/intrinsic.f, intrinsic.cy/intrinsic.f };
	
	for (std::vector<Vec3d>::const_iterator itr = from.begin(); itr != from.end(); itr++) {
		const double* uvz = itr->val;
		to.push_back(Vec3d(
			uvz[2]*(uvz[0]*invk[0]-invk[1]),
			uvz[2]*(uvz[1]*invk[0]-invk[2]),
			uvz[2]));
	}

	return to;
	
}

}

