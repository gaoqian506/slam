
#include "SpaceToolbox.h"
#include <assert.h>

namespace ww {


std::vector<Vec3d> SpaceToolbox::unproject(const std::vector<Vec3d>& from) {

	assert(0);
	return std::vector<Vec3d>();
	
}

std::vector<Vec3d> SpaceToolbox::unproject(const Intrinsic& intrinsic, const std::vector<Vec3d>& from) {

	std::vector<Vec3d> to;
	double invf = 1/intrinsic.f;
	//double invk[3] = { 1/intrinsic.f, intrinsic.cx/intrinsic.f, intrinsic.cy/intrinsic.f };
	
	for (std::vector<Vec3d>::const_iterator itr = from.begin(); itr != from.end(); itr++) {
		const double* uvz = itr->val;
		to.push_back(Vec3d(
			uvz[2]*invf*(uvz[0]-intrinsic.cx),
			uvz[2]*invf*(uvz[1]-intrinsic.cy),
			uvz[2]));
	}

	return to;
	
}

}

