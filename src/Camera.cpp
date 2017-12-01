

#include "Camera.h"

#include "MatrixToolbox.h"
#include <assert.h>

namespace ww {

Camera::Camera() : radius(20), movement(0) {

	rotation[0] = 1;
	rotation[4] = 1;
	rotation[8] = 1;

	original = NULL;
	gray = NULL;
	gradient[0] = NULL;
	gradient[1] = NULL;
	depth = NULL;
	points = NULL;
	depth_weight = NULL;
	//residual = NULL;
}

Camera::~Camera() {


}


Vec3f Camera::unproject(const float& u, const float& v, const float& d) {
	assert(0);
	return Vec3f();
}

Vec3f Camera::project(const Vec3f& pt) {
	assert(0);
	return Vec3f();

}

void Camera::rotation_warp(Image*& out, bool inverse/* = true*/) {

	if (!gray) { return; }
	double K[9] = {
		intrinsic.f, 0, intrinsic.cx,
		0, intrinsic.f, intrinsic.cy,
		0, 0, 1
	};
	double f1 = 1.0 / intrinsic.f;
	double iK[9] = {
		f1, 0, -intrinsic.cx*f1,
		0, f1, -intrinsic.cy*f1,
		0, 0, 1
	};
	Vec9d R = rotation;
	if (inverse) {
		R = MatrixToolbox::transpose_3x3(rotation);
	}
	Vec9d H = MatrixToolbox::mult_matrix_3x3(R, iK);
	H = MatrixToolbox::mult_matrix_3x3(K, H);
	gray->warp(H, out);

}

} // namespace
