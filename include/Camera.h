
#ifndef __WW_CAMERA_HEADER__
#define __WW_CAMERA_HEADER__



namespace ww {

struct Camera {

	Camera() : radius(10) {
	
	}

	double pos[3];
	double rotation[3];
	double radius;
};


}

#endif
