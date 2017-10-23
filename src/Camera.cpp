

#include "Camera.h"

namespace ww {

Camera::Camera() : radius(10) {
	original = NULL;
	gray = NULL;
	gradient[0] = NULL;
	gradient[1] = NULL;
	depth = NULL;
	residual = NULL;
}

Camera::~Camera() {


}


Vec3f Camera::unproject(const float& u, const float& v, const float& d) {

	

	return Vec3f();
}

Vec3f Camera::project(const Vec3f& pt) {
	return Vec3f();

}

} // namespace
