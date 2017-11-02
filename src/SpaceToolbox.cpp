
#include "SpaceToolbox.h"
#include <assert.h>
#include <math.h>

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

void SpaceToolbox::init_intrinsic(Intrinsic& intri, double fovy, int width, int height) {

	double theta = fovy / 180 * M_PI;
	intri.f = height * 0.5 / tan(0.5*theta);
	intri.cx = width * 0.5;
	intri.cy = height * 0.5;

}

} // namespace

