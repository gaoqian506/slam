

#include "Camera.h"

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

} // namespace
