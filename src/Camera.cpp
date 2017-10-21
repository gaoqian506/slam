

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

} // namespace
