
#ifndef __WW_CAMERA_HEADER__
#define __WW_CAMERA_HEADER__


#include "Image.h"
#include "Vectors.h"

namespace ww {


struct Intrinsic {
	Intrinsic() : f(500), cx(320), cy(240) {
	
	}
	double f;
	double cx;
	double cy;
};

class Camera {

public:
	Camera();
	~Camera();
	
	Vec3f unproject(const float& u, const float& v, const float& d);
	Vec3f project(const Vec3f& pt);

	Vec3d pos;
	Vec3d rotation;
	double radius;
	Intrinsic intrinsic;
	
	Image* original;
	Image* gray;
	Image* gradient[2];
	Image* depth;
	//Image* residual;
};


}

#endif
