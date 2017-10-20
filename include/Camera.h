
#ifndef __WW_CAMERA_HEADER__
#define __WW_CAMERA_HEADER__



namespace ww {


struct Intrinsic {
	Intrinsic() : f(500), cx(320), cy(240) {
	
	}
	double f;
	double cx;
	double cy;
};

struct Camera {

	Camera() : radius(10) {
	
	}

	double pos[3];
	double rotation[3];
	double radius;
	Intrinsic intrinsic;
};


}

#endif
